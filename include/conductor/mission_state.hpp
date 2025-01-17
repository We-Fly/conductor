#ifndef MISSION_STATE_H
#define MISSION_STATE_H

// 安全的SIGINT处理函数,能够紧急自动降落
void safeSigintHandler(int sig);

// 飞行任务状态
enum class MissionState
{
    kPrearm,
    kArm,
    kTakeoff,
    kPose,
    kLand,
    kTargetArmor,
    kTargetBridge,
    kTargetFort,
    kReturnPose,
    kWaybackHome
};

#endif // MISSION_STATE_H