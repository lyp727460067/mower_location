
neptune = {
  imu_instrinsic={
    bax = 0.0289 ,
    bay =0.021175,
    baz = -0.436975,

    nbax =0.00323   ,
    nbay =  0.00195,
    nbaz =  0.007267 ,
 
    kax =1.010950, 
    kay = 1.0036,
    kaz =1.0162750 ,

    tax = 0.0,
    tay =0.00125,
    taz = 0.0,

    bgx = 0.0065333,
    bgy = -0.02363333,
    bgz = 0.0120,

   

    nbgx =0.0001527525,
    nbgy = 0.000057735,
    nbgz = 0.00435889,
 
    kgx = 1.0069666,
    kgy =1.001166666,
    kgz =1.0008 ,
 
    tgx = -0.00015355,
    tgy =-0.002966666 ,
    tgz = 0.0010194,
    tgx1= -0.0000952,
    tgy1=  0.00293333,
    tgz1=  0.00243333,
  },
 sensor_extrinsic = {
   imu_to_gps ={
    x = 0.1,
    y = 0.1,
    z = 0.1,
    r = 0.1,
    p = 0.1,
    yaw = 0.1,
  },  
    imu_to_odom ={
     x = 0.1,
     y = 0.1,
     z = 0.1,
     r = 0.1,
     p = 0.1,
     yaw = 0.1,
    }, 
    bady_to_imu ={
     x = 0.1,
     y = 0.1,
     z = 0.1,
     r = 0.1,
     p = 0.1,
     yaw = 0.1,
   }, 
  },
  fusion_option={
    location_use_type =2,
    local_pose_option={
        fustion_type=1,
        fix_weitht=0.5,
        extraplaton_weitht=1,
    },
  }

}
return neptune