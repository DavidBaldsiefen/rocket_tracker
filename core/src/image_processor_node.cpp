#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv4/opencv2/opencv.hpp>
#include <ros/ros.h>
#include <torch/script.h>
#include <torch/torch.h>

static torch::jit::script::Module module;
static torch::Device torchDevice = torch::Device(torch::kCPU);

bool init() {

    // TODO: Change weightfile path based on CUDA availability
    static const std::string WEIGHTFILE_PATH =
        "/home/david/.ros/weightfiles/yolov5s.torchscript.pt";
    try {
        // Deserialize the ScriptModule from a file using torch::jit::load().
        module = torch::jit::load(WEIGHTFILE_PATH);
    } catch (const c10::Error &e) {
        ROS_ERROR("Could not load module from %s", WEIGHTFILE_PATH.c_str());
        return false;
    }

    ROS_INFO("Model/weightfile loaded from %s", WEIGHTFILE_PATH.c_str());

    if (torch::cuda::is_available()) {
        torchDevice = torch::Device(torch::kCUDA);
        module.to(torchDevice);
        ROS_INFO("Using CUDA Device for YOLOv5");
    } else {
        ROS_INFO("Using CPU for YOLOv5");
    }

    return true;
}

std::vector<torch::Tensor> non_max_suppression(torch::Tensor preds, float score_thresh = 0.5,
                                               float iou_thresh = 0.5,
                                               torch::Device device = torch::Device(torch::kCPU)) {
    std::vector<torch::Tensor> output;
    for (size_t i = 0; i < preds.sizes()[0]; ++i) {
        torch::Tensor pred = preds.select(0, i);

        // Filter by scores
        torch::Tensor scores =
            pred.select(1, 4) * std::get<0>(torch::max(pred.slice(1, 5, pred.sizes()[1]), 1));
        pred = torch::index_select(pred, 0, torch::nonzero(scores > score_thresh).select(1, 0));
        if (pred.sizes()[0] == 0)
            continue;

        // (center_x, center_y, w, h) to (left, top, right, bottom)
        pred.select(1, 0) = pred.select(1, 0) - pred.select(1, 2) / 2;
        pred.select(1, 1) = pred.select(1, 1) - pred.select(1, 3) / 2;
        pred.select(1, 2) = pred.select(1, 0) + pred.select(1, 2);
        pred.select(1, 3) = pred.select(1, 1) + pred.select(1, 3);

        // Computing scores and classes
        std::tuple<torch::Tensor, torch::Tensor> max_tuple =
            torch::max(pred.slice(1, 5, pred.sizes()[1]), 1);
        pred.select(1, 4) = pred.select(1, 4) * std::get<0>(max_tuple);
        pred.select(1, 5) = std::get<1>(max_tuple);

        torch::Tensor dets = pred.slice(1, 0, 6);

        torch::Tensor keep = torch::empty({dets.sizes()[0]}, device);
        torch::Tensor areas =
            (dets.select(1, 3) - dets.select(1, 1)) * (dets.select(1, 2) - dets.select(1, 0));
        std::tuple<torch::Tensor, torch::Tensor> indexes_tuple =
            torch::sort(dets.select(1, 4), 0, 1);
        torch::Tensor v = std::get<0>(indexes_tuple);
        torch::Tensor indexes = std::get<1>(indexes_tuple);
        int count = 0;
        while (indexes.sizes()[0] > 0) {
            keep[count] = (indexes[0].item().toInt());
            count += 1;

            // Computing overlaps
            torch::Tensor lefts = torch::empty(indexes.sizes()[0] - 1, device);
            torch::Tensor tops = torch::empty(indexes.sizes()[0] - 1, device);
            torch::Tensor rights = torch::empty(indexes.sizes()[0] - 1, device);
            torch::Tensor bottoms = torch::empty(indexes.sizes()[0] - 1, device);
            torch::Tensor widths = torch::empty(indexes.sizes()[0] - 1, device);
            torch::Tensor heights = torch::empty(indexes.sizes()[0] - 1, device);
            for (size_t i = 0; i < indexes.sizes()[0] - 1; ++i) {
                lefts[i] = std::max(dets[indexes[0]][0].item().toFloat(),
                                    dets[indexes[i + 1]][0].item().toFloat());
                tops[i] = std::max(dets[indexes[0]][1].item().toFloat(),
                                   dets[indexes[i + 1]][1].item().toFloat());
                rights[i] = std::min(dets[indexes[0]][2].item().toFloat(),
                                     dets[indexes[i + 1]][2].item().toFloat());
                bottoms[i] = std::min(dets[indexes[0]][3].item().toFloat(),
                                      dets[indexes[i + 1]][3].item().toFloat());
                widths[i] =
                    std::max(float(0), rights[i].item().toFloat() - lefts[i].item().toFloat());
                heights[i] =
                    std::max(float(0), bottoms[i].item().toFloat() - tops[i].item().toFloat());
            }
            torch::Tensor overlaps = widths * heights;

            // FIlter by IOUs
            torch::Tensor ious =
                overlaps /
                (areas.select(0, indexes[0].item().toInt()) +
                 torch::index_select(areas, 0, indexes.slice(0, 1, indexes.sizes()[0])) - overlaps);
            indexes = torch::index_select(indexes, 0,
                                          torch::nonzero(ious <= iou_thresh).select(1, 0) + 1);
        }

        keep = keep.toType(torch::kInt64);
        output.push_back(torch::index_select(dets, 0, keep.slice(0, 0, count)));
    }
    return output;
}

void processImage(cv::UMat frame) {
    // Step 1: Rescale Image

    // Preparing input tensor
    int width = 640, height = 640;
    cv::Mat img;
    cv::resize(frame, img, cv::Size(width, height)); // TODO: Configurable scaling
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);

    // Step 2:
    torch::Tensor imgTensor =
        torch::from_blob(img.data, {img.rows, img.cols, img.channels()}, torch::kByte)
            .to(torchDevice);
    imgTensor = imgTensor.permute({2, 0, 1});
    imgTensor = imgTensor.toType(torch::kFloat);
    imgTensor = imgTensor.div(255);
    imgTensor = imgTensor.unsqueeze(0);

    torch::Tensor preds = module.forward({imgTensor}).toTuple()->elements()[0].toTensor();

    std::vector<torch::Tensor> dets = non_max_suppression(preds, 0.4, 0.5, torchDevice);

    // Step 3: Evaluate detected objects

    if (dets.size() > 0) {

        int mostLikelyTarget = 0;
        float highestScore = 0.0;

        for (int i = 0; i < dets[0].sizes()[0]; i++) {
            float score = dets[0][i][4].item().toFloat();
            if (score > highestScore) {
                highestScore = score;
                mostLikelyTarget = i;
            }
        }

        // Draw the highest scoring target
        float left = dets[0][mostLikelyTarget][0].item().toFloat() * frame.cols / width;
        float top = dets[0][mostLikelyTarget][1].item().toFloat() * frame.rows / height;
        float right = dets[0][mostLikelyTarget][2].item().toFloat() * frame.cols / width;
        float bottom = dets[0][mostLikelyTarget][3].item().toFloat() * frame.rows / height;
        int classID = dets[0][mostLikelyTarget][5].item().toInt();

        // calculate boundingrect

        ROS_INFO("Detected Object of class: %d (%f%%)", classID, highestScore);
    }
}

void callbackFrameGrabber(const sensor_msgs::ImageConstPtr &msg) {
    cv_bridge::CvImageConstPtr img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    if (!img->image.empty()) {
        cv::UMat frame_umat = img->image.clone().getUMat(cv::ACCESS_RW); // TODO: Flag?

        // TODO: sync frame_ids to detected coordinates
        processImage(frame_umat);
    } else {
        ROS_WARN("Empty Frame received in image_processor_node::callbackFrameGrabber");
    }
}

int main(int argc, char **argv) {
    if (!init()) {
        ros::shutdown();
        return 0;
    }

    ros::init(argc, argv, "FRAMEGRABBER");
    ros::NodeHandle nh("~");

    // Creating image-transport subscriber
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber subimg = it.subscribe("/image_topic", 1, &callbackFrameGrabber);

    // Main loop
    ros::Rate r(100.0); // TODO: Set loop rate
    while (ros::ok()) {

        ros::spinOnce();
        r.sleep();
    }

    // Shut everything down cleanly
    ros::shutdown();
    subimg.shutdown();
    nh.shutdown();
    return 0;
}
