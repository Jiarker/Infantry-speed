import matplotlib.pyplot as plt
import os

predict_speed = []
predict_time = []

now_speed = []
now_time = []

path = './src/Algorithm/configure/Detector/Fitting/buff_state/'
file_list = os.listdir(path)

# 得到文件夹中文件数最大的文件名
max_index = 0
max_file_name = '0.txt'
for file_name in file_list:
    if file_name.endswith(".txt"):
        index = int(file_name.split(".")[0])
        if index > max_index:
            max_index = index
            max_file_name = file_name

print(max_file_name)

max_file_path = path + '/' + max_file_name


with open(max_file_path , "r") as speed_txt:
    for line in speed_txt.readlines():
        curLine = line.strip().split(" ")
        floatLine = map(float,curLine) 
        floatLine = list(floatLine)

        predict_speed.append(floatLine[0])
        predict_time.append(floatLine[1])

        now_speed.append(floatLine[2])
        now_time.append(floatLine[3])


plt.scatter(predict_time, predict_speed, s=5, c='blue') # 蓝色表示预测
plt.scatter(now_time, now_speed, s=5, c='red') # 红色表示真实

plt.show()