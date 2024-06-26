'''
Created on Dec 10, 2017

Run a simulation with the trigger defined in Defs.py. After 10s the first vehicle in the platoon will start to brake.

@author: Dominik Baumann
MPI-IS, ICS
dbaumann(at)tuebingen.mpg.de
'''

from VehiclePlatoon import *

#          num_it：仿真时间 25秒
#          num_it：仿真时间 25秒
#          传感器采样间隔 0.1
#          num_veh：车辆个数
def run_simulation_with_braking(trigger):
    with open('trriger.txt', 'w') as f:
          f.write(str(trigger))
    # 初始化 车辆队列 和 车辆状态信息(本地信息，id，状态，控制信号，滤波器，本地触发器等)
    exp_platoon = platoon(x_init)
    vehicles = [vehicle(x_des_init, x_init, i, delta) for i in range(num_veh)]
    # 画图的矩阵
    state_out = np.zeros((2 * num_veh, num_it))
    comm_out = np.zeros((num_veh, num_it))
    t = np.arange(0, num_it * Ts, Ts)
    # 初始化控制输入信号 num_veh：车辆个数
    u = np.zeros((num_veh, 1))
    # 噪声 v:过程噪声，w:传感器噪声
    np.random.seed(12122017)
    v = np.random.uniform(-v_max, v_max, (num_veh, num_it))
    w = np.random.uniform(-w_max, w_max, (num_veh, num_it))
    for i in range(0, num_it):
        # 先更新当前时间戳的车辆传感器的测量值 y(t) ,并检查车辆是否相撞
        y = exp_platoon.propagate(u, v[:, i][np.newaxis].T, w[:, i][np.newaxis].T)
        if exp_platoon.check_for_accidents():
            print('accident')
            break

        # 当前状态 （用于画图）
        state_out[:, i] = exp_platoon.state[:, 0]

        index = 0
        # 第十秒，头车制动，速度设置为0
        if i > 100:
            for j in range(0, num_veh):
                vehicles[0].x_des[2 * j ] = 0

        # 车辆传感器决策是否触发通信
        for obj in vehicles:
            # 估计本地状态并预测全局的车辆状态 predict()预测全局车辆状态
            # estimate_state()获得卡尔曼滤波器对当前车辆估计，并在全局矩阵中进行替换
            obj.predict()
            obj.estimate_state(y)

            # 每辆车的本地控制信号输入
            u[obj.veh_id, 0] = obj.u[obj.veh_id, 0]
            # trigger  1：PT  0：ST
            # 检查两种触发器的的触发条件gamma_pred()
            if trigger == 1:
                obj.gamma_pred(i)
            else:
                obj.gamma_self()

        # 轮询车辆，如果该车辆的触发值gamma为1，则触发车辆状态消息传递
        for obj in vehicles:
            if obj.gamma == 1:
                # Safe triggering decision
                comm_out[index, i] = 1
                # 触发->重置卡尔曼滤波器的协方差矩阵等
                obj.state_loc_pred = copy.deepcopy(obj.state_loc)
                obj.state_pred = copy.deepcopy(obj.state)
                obj.x_des_pred = copy.deepcopy(obj.x_des)
                obj.klm_loc_pred.Pol = copy.deepcopy(obj.klm_loc.Pcl)
                for veh in vehicles:
                    # 如果没有数据包丢失，则将车辆状态信息传递给所有其他车辆
                    # 丢包率为0.1 随机产生一个值 >0.1 就算没丢包
                    if np.random.uniform() > pdr:
                        veh.state_pred[2 * index, 0] = obj.state[2 * index, 0]
                        veh.state_pred[2 * index + 1, 0] = obj.state[2 * index + 1, 0]
                        veh.state[2 * index, 0] = obj.state[2 * index, 0]
                        veh.state[2 * index + 1, 0] = obj.state[2 * index + 1, 0]
                        veh.x_des = copy.deepcopy(obj.x_des)
                        veh.x_des_pred = copy.deepcopy(obj.x_des)
                        # 车辆控制信号
                        u[veh.veh_id, 0] = veh.calc_input(veh.state, veh.x_des)[veh.veh_id, 0]
            index += 1


    mpl.subplot(2, 1, 1)
    mpl.ylim((20,35))

    mpl.xlabel("t(s)")
    mpl.ylabel("relative distance")
    # Can plot either vehicle distances or state of vehicles
    l1 = mpl.plot(t, (state_out[0, :] - state_out[2, :]).transpose())
    l2 = mpl.plot(t, (state_out[2, :] - state_out[4, :]).transpose())
    mpl.legend(labels = ['relative distance between node2,node1','relative distance between node3,node2'])
    mpl.grid()
    mpl.subplot(2, 1, 2)
    mpl.ylim((0, 1.5))
    mpl.xlabel("t(s)")
    mpl.ylabel("status broadcast triggered")
    mpl.plot(t, comm_out.transpose())
    mpl.legend(labels=['node0', 'node1','node2'])
    mpl.grid()
    mpl.show()

    print('predict distance', (state_out[0, :] - state_out[2, :]).transpose())
    print('position state of veh0', (state_out[0, :]).transpose())
    print('position state of veh1', (state_out[2, :]).transpose())
    print('speed state of veh0', (state_out[:, 0]).transpose())
    print('speed state of veh1', (state_out[:, 2]).transpose())

    dece = (state_out[:, 2])
#    dece = [x.strip() for x in dece if x.strip() != '']

    t = (2 * dist1 + ((int(dece[2])) ^ 2)) / des_vel
    with open('deceleration.txt', 'w') as f:
          f.write(str(t))
     # S=v*t-（a*t^2）/2


if __name__ == '__main__':

   run_simulation_with_braking(0)
   
