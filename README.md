# Path-Planning-Wireless-Charging
Adaptive Multi-UAV Path Planning and Mobile Charging Coordination in UAV-Assisted MEC Networks
Unmanned aerial vehicle (UAV)-assisted multi-access edge computing (MEC) has emerged as
a promising solution for providing flexible and low-latency computation services in dynamic
and infrastructure-limited environments. However, the performance of such systems is
fundamentally constrained by UAV energy limitations, complex path planning requirements,
and the need for continuous adaptation to heterogeneous user demands. This paper proposes
an integrated reinforcement learning–based framework for joint path planning and wireless
charging optimization in multi-UAV-assisted MEC networks. The problem is formulated as a
Markov Decision Process, in which UAVs act as learning agents that autonomously optimize
their trajectories, charging decisions, and service provisioning policies based on
environmental risk, terminal user demand, and energy state information. A cooperative
architecture with mobile charging UAVs is introduced to enable in-flight wireless energy
replenishment and sustained mission execution. Since the battery-limited UAVs cannot
accomplish all tasks and maintain mission flight without recharging, we integrate the mobile
charging UAVs (CUAV) into the environment map. Decentralized mobile charging stations
are deployed to sustain UAV operations and enable long-term data offloading, where the
UAVs can recharge their energy and prolong the mission flight. Furthermore, a QoS-aware
sigmoid demand model is incorporated into the reward design to improve service
prioritization. Extensive simulation results demonstrate that the proposed framework
significantly reduces mission completion time, minimizes unnecessary charging events,
maintains stable energy safety margins, and scales effectively with increasing operational
area, outperforming baseline strategies in both efficiency and robustness.
