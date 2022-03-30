#!/usr/bin/env python
from dis import dis
from math import dist
import matplotlib.pyplot as plt
import csv

odom_x=[]
odom_y=[]
odom_or_x=[]
odom_or_y=[]
odom_or_z=[]
odom_or_w=[]


part_x=[]
part_y=[]
part_or_x=[]
part_or_y=[]
part_or_z=[]
part_or_w=[]

time1=[]
time2=[]
odom_file=open('bags/sim_noise/odom_noise.txt','r')
#odom_file=open('first_sim/odom_sim_1.txt','r')
odom_plt = csv.reader(odom_file,delimiter=',')


part_file=open('bags/sim_noise/particle_noise.txt','r')
part_plot = csv.reader(part_file,delimiter=',')



time_break1='1648609425697471900'
not_add=True

for row in odom_plt:
    if row[0]=='%time':
        continue
    if row[0]==time_break1:
        not_add=False
        continue
    if not_add:
        continue
    time1.append(float(row[0]))

    odom_x.append(float(row[5]))
    odom_y.append(float(row[6]))
    odom_or_x.append(float(row[8]))
    odom_or_y.append(float(row[9]))
    odom_or_z.append(float(row[10]))
    odom_or_w.append(float(row[11]))


time_break2='1648609425683782300'
not_add2=True
for row in part_plot:
    if row[0]=='%time':
        continue
    if row[0]==time_break2:
        not_add2=False
        continue
    
    if not_add2:
        continue
    time2.append(float(row[0]))

    part_x.append(float(row[5]))
    part_y.append(float(row[6]))
    part_or_x.append(float(row[8]))
    part_or_y.append(float(row[9]))
    part_or_z.append(float(row[10]))
    part_or_w.append(float(row[11]))

error=[]
timer=[]

first_time1=time1[0]
first_time2=time2[0]

for t in range(len(time1)):
    time1[t]=(time1[t]-first_time1)*10**(-9)

for t in range(len(time2)):
    time2[t]=(time2[t]-first_time2)*10**(-9)


for t in range(len(time2)):
    for  k in range(len(time1)):
        #if (((time1[k]-time2[t])**2)**0.5)>=0.5:
        #    break
        if time1[k]<time2[t]+0.01 and time1[k]>time2[t]-0.01:
            error_is= ((part_x[t]-odom_x[k])**2 +(part_y[t]-odom_y[k])**2)**0.5
            break

    if time2[t]>28:
        break
    error.append(error_is)
    timer.append(time2[t])

print(len(time1))
print(len(time2))


#print(sum(error)/len(error))
#print(len(time1))
#print(time1[100])
#print('_________________________')
#print(len(time2))
#print(time2[100])

#print(error)
#plt.plot(odom_x,odom_y,label="Actual Pose")
#plt.plot(part_x,part_y,label="Particle Filter Estimate")
plt.plot(timer,error,label ='Average Error= '+str(round(sum(error)/len(error),3)))

plt.title('Larger /odom Noise Error')
#plt.xlabel('x (m)')
#plt.ylabel('y (m)')
#plt.xlabel('time')
#plt.ylabel('error')
plt.legend(bbox_to_anchor=(1.1,1.1))
plt.show()