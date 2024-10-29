#include <random>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/utils.h>
#include <rv2_interfaces/qos.h>

#define DEFAULT_SERVICE_NAME "qos_default"

static std::atomic<bool> __globalExitFlag = false;

enum QoSContentEnum { TOPIC_NAME, QOS_TYPE, DEPTH, RELIABILITY, DURABILITY };

std::string GenQoSContent(QoSContentEnum type)
{
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::uniform_int_distribution<> uniInt26Distrib{0, 25};
    std::string devType[] = { "publisher", "subscription", "both" };

    std::string ret = "";
    switch (type)
    {
    case QoSContentEnum::TOPIC_NAME:
        ret += "/V0/";
        for (int i = 0; i < 8; i++)
            ret += uniInt26Distrib(gen_) + 97;
        ret += "_" + std::to_string(uniInt26Distrib(gen_));
        break;
    case QoSContentEnum::QOS_TYPE:
        ret = std::to_string((uniInt26Distrib(gen_) % 3) + 1);
        break;
    case QoSContentEnum::DEPTH:
        ret = std::to_string(uniInt26Distrib(gen_) % 11);
        break;
    case QoSContentEnum::RELIABILITY:
        ret = std::to_string(uniInt26Distrib(gen_) % 4);
        break;
    case QoSContentEnum::DURABILITY:
        ret = std::to_string(uniInt26Distrib(gen_) % 4);
        break;
    default:
        break;
    }
    return ret;
}

void PrintHelp()
{
    printf(
"/**\n\
 * Profile Registration\n\
 * - Profile add (determine (1)publisher, (2)subscription or (3)both):\n\
 *   - pr a <topic_name> <1/2/3>\n\
 *   - pr a <topic_name> <1/2/3> <depth>\n\
 *   - pr a <topic_name> <1/2/3> <depth> <reliability_enum>\n\
 *   - pr a <topic_name> <1/2/3> <depth> <reliability_enum> <durability_enum>\n\
 * - Profile remove:\n\
 *   - pr r <topic_name>\n\
 * - Profile clear:\n\
 *   - pr c\n\
 * - Profile save:\n\
 *   - pr s\n\
 * \n\
 * Profile Request\n\
 * - Profile request (determine (1)publish or (2)subscription):\n\
 *   - prq <topic_name> <p/s>\n\
 *   - prq all\n\
 * \n\
 * Ignore item: '-' \n\
 * \n\
 * Server Control\n\
 * - Qmap (S)ave / (R)emove:\n\
 *   - c qs\n\
 *   - c qr\n\
 * \n\
 * - Topic device map (S)ave / (R)emove:\n\
 *   - c ts\n\
 *   - c tr\n\
 * \n\
 * - Re-register signal:\n\
 *   - c r\n\
 *   - c ra\n\
 * \n\
 * Quit: \n\
 *   !\n\
 * Help: \n\
 *   ?\n\
 */\n"
    );
}

int main(int argc, char* argv[])
{
    // Ctrl-c handler
    signal(SIGINT, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGTERM handler
    signal(SIGTERM, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    // SIGKILL handler
    signal(SIGKILL, [](int signum) {
        (void)signum;
        __globalExitFlag.store(true);
    });

    rclcpp::init(argc, argv);
    auto controlNode = std::make_shared<rclcpp::Node>("qoscontrol_default");
    std::string serviceName = DEFAULT_SERVICE_NAME;
    rv2_interfaces::GetParam(controlNode, "qosService", serviceName, serviceName, "qosService: ", false);
    auto execNode = std::make_shared<rv2_interfaces::ExecNode>(controlNode);
    execNode->start();

    std::cout << "Service name: " << serviceName << std::endl;
    PrintHelp();

    while (!__globalExitFlag.load())
    {
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);

        if (inputStr.size() <= 0)
            continue;
        auto inputStrVec = rv2_interfaces::split(inputStr, ", ");
        if (inputStrVec[0] == "!")
        {
            __globalExitFlag.store(true);
            break;
        }
        else if (inputStrVec[0] == "?")
        {
            PrintHelp();
        }
        else if (inputStrVec[0] == "pr" && inputStrVec.size() >= 2)
        {
            auto req = std::make_shared<rv2_interfaces::srv::TopicQosProfileReg::Request>();
            auto &tQoS = req->topic_qos_profile;

            if (inputStrVec[1] == "c")
            {
                req->clear_profiles = true;
            }
            else if (inputStrVec[1] == "s")
            {
                req->save_qmap = true;
            }
            else if (inputStrVec[1] == "a")// pr a
            {
                if (inputStrVec.size() >= 3)
                    tQoS.topic_name = inputStrVec[2] == "-" ? GenQoSContent(TOPIC_NAME) : inputStrVec[2];
                else
                    tQoS.topic_name = GenQoSContent(TOPIC_NAME);

                if (inputStrVec.size() >= 4)
                {
                    tQoS.qos_type = std::stoi(inputStrVec[3] == "-" ? GenQoSContent(QOS_TYPE) : inputStrVec[3]);

                }
                else
                    tQoS.qos_type = std::stoi(GenQoSContent(QOS_TYPE));

                rv2_interfaces::msg::QosProfile prof;

                if (inputStrVec.size() >= 5)
                    prof.depth = std::stoi(inputStrVec[4] == "-" ? GenQoSContent(DEPTH) : inputStrVec[4]);
                else
                    prof.depth = std::stoi(GenQoSContent(DEPTH));

                if (inputStrVec.size() >= 6)
                    prof.reliability = std::stoi(inputStrVec[5] == "-" ? GenQoSContent(RELIABILITY) : inputStrVec[5]);
                else
                    prof.reliability = std::stoi(GenQoSContent(RELIABILITY));

                if (inputStrVec.size() >= 7)
                    prof.durability = std::stoi(inputStrVec[6] == "-" ? GenQoSContent(DURABILITY) : inputStrVec[6]);
                else
                    prof.durability = std::stoi(GenQoSContent(DURABILITY));
                tQoS.qos_profile = prof;
            }
            else if (inputStrVec[1] == "r" && inputStrVec.size() == 3)
            {
                tQoS.topic_name = inputStrVec[2];
                req->remove_profile = true;
            }
            else
                continue;

            std::cout << "Request: " << tQoS.topic_name << " " << rv2_interfaces::GetQosTypeStr(tQoS.qos_type) << " " << req->clear_profiles << " " << req->save_qmap << " " << req->remove_profile << std::endl;
            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::TopicQosProfileReg>(controlNode, QoSServer_TopicQosProfileRegSrvName(serviceName), req, 500ms);
            if (res)
                std::cout << "Response: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
            else
                std::cerr << "Request failed: " << QoSServer_TopicQosProfileRegSrvName(serviceName) << std::endl;
        }
        else if (inputStrVec[0] == "prq" && inputStrVec.size() >= 3)
        {
            auto req = std::make_shared<rv2_interfaces::srv::TopicQosProfileReq::Request>();
            req->topic_name = inputStrVec[1];
            if (inputStrVec[2] == "1")
                req->qos_type = rv2_interfaces::msg::TopicQosProfile::QOS_TYPE_PUBLISHER;
            else if (inputStrVec[2] == "2")
                req->qos_type = rv2_interfaces::msg::TopicQosProfile::QOS_TYPE_SUBSCRIPTION;
            else if (inputStrVec[2] == "3")
                req->qos_type = rv2_interfaces::msg::TopicQosProfile::QOS_TYPE_BOTH;
            else
                continue;
            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::TopicQosProfileReq>(controlNode, QoSServer_TopicQosProfileReqSrvName(serviceName), req, 500ms);
            if (res)
            {
                std::cout << "Response: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
                if (res->response != rv2_interfaces::ServiceResponseStatus::SRV_RES_SUCCESS && 
                    res->response != rv2_interfaces::ServiceResponseStatus::SRV_RES_IGNORED && 
                    res->response != rv2_interfaces::ServiceResponseStatus::SRV_RES_WARNING)
                    continue;

                rv2_interfaces::HierarchicalPrint hprint;
                for (const auto& tQoS : res->topic_qos_profile_vec)
                    rv2_interfaces::AddHierarchicalPrint(hprint, 0, tQoS);
                hprint.print();
                hprint.clear();

            }
            else
                std::cerr << "Request failed" << std::endl;

        }
        else if (inputStrVec[0] == "c" && inputStrVec.size() >= 2)
        {
            auto req = std::make_shared<rv2_interfaces::srv::QosServer::Request>();
            if (inputStrVec[1] == "qs")// Save qmap signal.
            {
                req->request.qmap_action = rv2_interfaces::msg::QosServerStatus::QMAP_ACTION_SAVE_FILE;
            }
            else if (inputStrVec[1] == "qr")// Remove qmap signal.
            {
                req->request.qmap_action = rv2_interfaces::msg::QosServerStatus::QMAP_ACTION_REMOVE_FILE;
            }
            else if (inputStrVec[1] == "ts")// Save topic device map signal.
            {
                req->request.topic_device_action = rv2_interfaces::msg::QosServerStatus::TOPIC_DEVICE_ACTION_SAVE_FILE;
            }
            else if (inputStrVec[1] == "tr")// Remove topic device map signal.
            {
                req->request.topic_device_action = rv2_interfaces::msg::QosServerStatus::TOPIC_DEVICE_ACTION_REMOVE_FILE;
            }
            else if (inputStrVec[1] == "r")// Re-register signal.
            {
                req->request.re_register_action = rv2_interfaces::msg::QosServerStatus::RE_REGISTER_ACTION_UNREGISTERED;
            }
            else if (inputStrVec[1] == "ra")// Re-register signal.
            {
                req->request.re_register_action = rv2_interfaces::msg::QosServerStatus::RE_REGISTER_ACTION_ALL;
            }
            else
                continue;

            auto res = rv2_interfaces::ClientRequestHelper<rv2_interfaces::srv::QosServer>(controlNode, QoSServer_QosServerSrvName(serviceName), req, 500ms);
            if (res)
                std::cout << "Response: " << rv2_interfaces::GetServiceResponseStatusMsg(res->response) << " Reason: " << res->reason << std::endl;
            else
                std::cerr << "Request failed" << std::endl;
        }
        std::this_thread::sleep_for(50ms);
    }
    rclcpp::shutdown();
}
