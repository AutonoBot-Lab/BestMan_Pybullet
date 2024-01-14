
--------------------------------------------------------------------
1. Find the pybullet installation path
    If you install Anaconda,the path will like that: /home/ubuntu/anaconda3/envs/<YOU_ENV_NAME>/lib/python3.8/site-packages/pybullet_data/
    If not, find it in you package
2. copy the usersPlane to the path you which you find

3. add the path in you code, like that:
    p.loadURDF("userPlane/<you want color package: eg:goldPlane>/plane.urdf")
    eg.
        p.loadURDF("userPlane/goldPlane/plane.urdf")
4. End
--------------------------------------------------------------------
    
