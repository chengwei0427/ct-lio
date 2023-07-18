import matplotlib.pyplot as plt

#   读取文本文件
with open('./log/log_time.txt', 'r') as f:
    data = f.readlines()

#   取第一行所有的字符串
data0 = [line.strip() for line in data if line.strip() and line.startswith('build frame')]
data0 = [[v.strip() for v in line.split(',')] for line in data0]
print(data0)
d = len(data0[0])-1
# print(len(data0[0]))
print("record time size: ")
print(d)

#   从第二行开始，读取所有数据
data = [line.strip() for line in data if line.strip() and not line.startswith('build frame')]
data = [[float(v) if v.strip() else None for v in line.split(',')] for line in data]
# print(data)

#   颜色数组
color_vec = ['blue','red','green','orange','purple','brown','pink','blueviolet','gold','yellow','gray','olivedrab','darkgoldenrod','forestgreen','skyblue']

#   获取每一列的数据
alldata=[]
i = 0
while i < d:
    gnss = [d[i] for d in data if d[i] is not None]
    alldata.append(gnss)
    i +=1

#   指定需要显示的数据
plot_list=[0,1,2,3,4,5,6,7,8,9,10]

#   画折线图 (un-comment bellow code for line image)
for i in plot_list:
    plt.plot(alldata[i],color_vec[i],label=data0[0][i])

#   显示网格
plt.grid(linestyle=":", color="r")


plt.legend(loc='best')

plt.xlabel('Sample')
plt.ylabel('Time(ms)')
plt.title('Time Comparison')

#   画箱线图 (un-comment bellow code for box image)
# data_for_box = []
# for i in plot_list:
#     data_for_box.append(alldata[i])

# label_box = [data0[0][d] for d in plot_list]

# plt.grid(True)
# plt.boxplot(data_for_box,
#             medianprops = {'color': 'red'}, #   中值
#             meanline = True,    
#             showmeans = True,
#             meanprops = {'color': 'blue', 'ls':'--'},# 均值
#             flierprops = {"marker": "*", "markerfacecolor": "red"}, # 异常值绘制
#             # vert = False,   #   横着还是竖着
#             # showfliers = False, #   是否显示异常值
#             patch_artist=True,boxprops={'facecolor': 'green', 'edgecolor': 'black'}, # box属性
#             labels = label_box)

plt.show()
