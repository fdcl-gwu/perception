import rosbag

print('get few bag msgs for test')
file_dir='/mnt/tlee_share/Videos/Vicon_octomap/'
file_name='_2017-03-29-18-11-22.bag'
file_out = 'test.bag'
num_msgs = 130
with rosbag.Bag(file_dir+file_out, 'w') as outbag:
    for topic, msg, t in rosbag.Bag(file_dir+file_name).read_messages():
        if num_msgs < 1:
            break
        num_msgs -=1
        outbag.write(topic, msg, t)
