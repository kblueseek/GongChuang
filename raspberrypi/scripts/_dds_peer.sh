#!/usr/bin/env bash
# 跨网段/跨 WiFi 时强制「单播发现」用的小工具，run_car.sh 和 run_gui.sh 都会 source 它。
#
# 为什么需要：ROS 2 默认靠组播（239.255.0.1）互相找，校园网、路由器隔离、
# 两个不同网段（比如 10.23.116.x 和 10.23.113.x）时组播根本过不去，
# 表现就是「ros2 node list 一个节点都没有」。
# 把对端 IP 写进 initialPeersList 后，两边直接用单播打招呼，绕开组播。
#
# 用法（在别的脚本里）：
#     source "$(dirname "$0")/_dds_peer.sh"
#     setup_dds_peer 10.23.113.193      # 传空串 = 什么都不做，走默认组播

setup_dds_peer() {
    local peer_ip="$1"

    # 没传 IP 就什么都不干：同一台机器上跑，或者组播本来就能通的时候用不着。
    if [ -z "$peer_ip" ]; then
        return 0
    fi

    local xml="$HOME/.ros/fastdds_unicast_peer.xml"
    mkdir -p "$HOME/.ros"

    cat > "$xml" <<EOF
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <participant profile_name="unicast_peer" is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator><udpv4><address>${peer_ip}</address></udpv4></locator>
                </initialPeersList>
            </builtin>
        </rtps>
    </participant>
</profiles>
EOF

    # 新版本 ROS2 认 FASTDDS_，旧版本认 FASTRTPS_，两个都设上最省心
    # （只设旧的那个，新版会打一堆 deprecated 警告）。
    export FASTDDS_DEFAULT_PROFILES_FILE="$xml"
    export FASTRTPS_DEFAULT_PROFILES_FILE="$xml"
    echo "[i] 已启用单播发现，对端 = ${peer_ip}"
}
