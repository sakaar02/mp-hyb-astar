#!/usr/bin/env python
import numpy as np
import math
from scipy import interpolate
from scipy.interpolate import interp1d

import rospy
import roslib 
from nav_msgs.msg import Path
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped


def Callback(msg):

  # x_org = np.array([msg.poses[i].pose.position.x for i in range(len(msg.poses))])
  # y_org = np.array([msg.poses[i].pose.position.y for i in range(len(msg.poses))])
  x_org = [round(msg.poses[i].pose.position.x, 1) for i in range(len(msg.poses))]
  y_org = [round(msg.poses[i].pose.position.y, 1) for i in range(len(msg.poses))]

  x_org.reverse()
  y_org.reverse()

  # https://stackoverflow.com/questions/46816099/scipy-interpolate-splrep-data-error

  x_org2=[]
  y_org2=[]

  for i in range(len(y_org)-1):
    if y_org[i]>=y_org[i+1]:
      continue
    else:
      x_org2.append(x_org[i])
      y_org2.append(y_org[i])      

      # y_org.pop(i+1)
      # x_org.pop(i+1)
  
  #print(y_org2)

  # print(len(msg.poses))
  # print(y_org.shape)

  # if (y_org.shape[0] > 0 and x_org.shape[0] > 0):
  if (len(y_org) > 0 and len(x_org) > 0):

    # spline = interpolate.splrep(x_org, y_org, s=0) #s is extra smoothening factor
    spline = interpolate.splrep(y_org2, x_org2, s=4, k=5) #s is extra smoothening factor
    # https://stackoverflow.com/questions/12054060/scipys-splrep-splev-for-python-interpolation-returns-nan
    # s=0 gave nans 


    # xnew = x_org
    # ynew = interpolate.splev(xnew, spline, der=0) #der is derivative of spline, i.e, zeroth derivative here
    xnew = interpolate.splev(y_org2, spline, der = 0) #der is derivative of spline, i.e, zeroth derivative here
    dxdy = interpolate.splev(y_org2, spline, der = 1) #this will give us dx/dy because the arrays are inverted in the first place
    d2xdy2 = interpolate.splev(y_org2, spline, der = 2)
    dydx = np.reciprocal(dxdy) #this is basically the tangent at any (x,y)

    #head_angle = np.vectorize(math.atan(dydx)) #throws error because atan accepts a single value, not an array
    head_angle = []
    d2ydx2 = []

    for i in range(len(y_org2)):
      d2ydx2.append(-(d2xdy2[i])*(dydx[i])**3)

    for i in range(len(y_org2)):
      head_angle.append(math.atan(dydx[i]))

    path = Path()
    path.header = Header()
    path.header.frame_id = "/map"

    # for i in range(len(msg.poses)):
    for i in range(len(y_org2)):
      
      vertex = PoseStamped()
      # vertex.header = std_msgs.msg.Header()
      vertex.header = Header()
      vertex.header.stamp = rospy.Time.now()
      vertex.header.frame_id="/map"

      vertex.pose.position.x= float(xnew[i])
      vertex.pose.position.y= float(y_org2[i])
      vertex.pose.position.z= 0.0

      vertex.pose.orientation.x= float(head_angle[i]) #angle
      vertex.pose.orientation.y= float(dydx[i]) #first derivative
      vertex.pose.orientation.z= float(d2ydx2[i]) #second derivative
      vertex.pose.orientation.w= 0.0

      path.poses.append(vertex)
        

    pub.publish(path)


def main():

  rospy.init_node('Smoother', anonymous=True)
  path_sub = rospy.Subscriber('/path', Path, Callback)

  global pub

  pub = rospy.Publisher("/Smooth_Hybrid_node", Path, queue_size=10)
  rate = rospy.Rate(10) # 10hz
  rate.sleep()
  rospy.spin()

if __name__ == '__main__':
  try:
    main()
  except rospy.ROSInterruptException:
    Pass
