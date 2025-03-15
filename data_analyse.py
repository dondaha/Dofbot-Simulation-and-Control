import re

states = [] # 实际关节角度
g = [] # 插值得到的目标关节角度
length = 0

with open("t.txt", 'r', encoding='utf-8') as file:
    while True:
        line1 = file.readline()
        line2 = file.readline()
        if not line1 or not line2:
            break
        
        state_values = re.findall(r"[-+]?\d*\.\d+|\d+", line1)
        goal_values = re.findall(r"[-+]?\d*\.\d+|\d+", line2)
        # print(state_values)
        # print(goal_values)
        
        states.append([float(value) for value in state_values])
        g.append([float(value) for value in goal_values])
        length += 1
        

# 补全空数据
last_goal = [-1, -1, -1, -1, -1, -1]
for i in range(len(states)):
    if len(g[i]) == 1:
        last_goal[5] = g[i][0]
    if len(g[i]) == 5:
        last_goal[0] = g[i][0]
        last_goal[1] = g[i][1]
        last_goal[2] = g[i][2]
        last_goal[3] = g[i][3]
        last_goal[4] = g[i][4]
    if last_goal[5] == -1:
        last_goal[5] = states[i][5]
    g[i] = last_goal.copy()
    if i==20:
        pass

# 输出结果
for i in range(len(states)):
    print("STATE:", states[i])
    print("GOAL :", g[i])
    
# 可视化states和g
import matplotlib.pyplot as plt
import numpy as np
# Convert lists to numpy arrays for easier manipulation
states_np = np.array(states)
g_np = np.array(g)

# Plot each joint angle over time
for joint in range(6):
    plt.figure()
    plt.plot(states_np[:, joint], label=f'State Joint {joint+1}')
    plt.plot(g_np[:, joint], label=f'Goal Joint {joint+1}', linestyle='--')
    plt.xlabel('Time Step')
    plt.ylabel('Angle (degrees)')
    plt.title(f'Joint {joint+1} Angles Over Time')
    plt.ylim(0, 180)  # Set y-axis range from 0 to 180
    plt.legend(loc='best', fontsize='small')  # Adjust legend location and size
    plt.show()

# Plot all joint angles on a single plot
plt.figure()
for joint in range(6):
    plt.plot(states_np[:, joint], label=f'State Joint {joint+1}')
    plt.plot(g_np[:, joint], label=f'Goal Joint {joint+1}', linestyle='--')
plt.xlabel('Time Step')
plt.ylabel('Angle (degrees)')
plt.title('All Joint Angles Over Time')
plt.ylim(0, 180)  # Set y-axis range from 0 to 180
plt.legend(loc='best', fontsize='small')  # Adjust legend location and size
plt.show()
