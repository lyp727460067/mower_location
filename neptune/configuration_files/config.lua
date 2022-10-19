neptune = {
    imu_instrinsic = {
        bax = 0.0289,
        bay = 0.021175,
        baz = -0.436975,

        nbax = 0.00323,
        nbay = 0.00195,
        nbaz = 0.007267,

        kax = 1.010950,
        kay = 1.0036,
        kaz = 1.0162750,

        tayz = 0.0, -- t01
        tazy = 0.00125, -- t02
        tazx = 0.0, -- t12

        bgx = 0.0065333,
        bgy = -0.02363333,
        bgz = 0.0120,

        nbgx = 0.0001527525,
        nbgy = 0.000057735,
        nbgz = 0.00435889,

        kgx = 1.0069666,
        kgy = 1.001166666,
        kgz = 1.0008,
        nbgx = 0.0001527525,
        nbgy = 0.000057735,
        nbgz = 0.00435889,

        kgx = 1.0069666,
        kgy = 1.001166666,
        kgz = 1.0008,

        tgyz = -0.00015355, -- t01
        tgzy = -0.002966666, -- t02
        tgxz = 0.0010194, -- t10
        tgzx = -0.0000952, -- t12
        tgxy = 0.00293333, -- t20
        tgyx = 0.00243333 -- t21
    },
    fusion_option = {
        location_use_type = 2,
        local_pose_option = {
            fustion_type = 1,
            fix_weitht_traslation= 10.1,
            fix_weitht_rotation  = 0;
            extraplaton_weitht = 0.2,
            slide_windows_num  =20,
        }
    },
    sensor_extrinsic = {
        imu_to_gps = {x = 0, y = 0.078, z = 0.093, r = 0, p = 0, yaw = 0},
        imu_to_odom = {x = 0.072, y = 0.395, z = 0.054, r = 0, p = 0, yaw = 0},
        -- imu_to_odom = {x = 0., y = 0., z = 0., r = 0, p = 0, yaw = 0},
        body_to_imu = {x = 0, y = 0, z = 0, r = 0, p = 0, yaw = 0}
    },
    kinamics_params = {
        b = 0.3817,
        nv = {x = 0.01, y = 0.01, z = 0.01},
        nw = {x = 0.01, y = 0.01, z = 0.01},
        r = 0.192
    }

}
return neptune
