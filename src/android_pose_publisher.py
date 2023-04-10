import rospy
import websocket
import json
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

class SensorDataClient():
    def __init__(this, url):
        this.url = url
        this.publisher = rospy.Publisher('android_pose', PoseStamped, queue_size=10)
        this.test = rospy.Publisher('test', String, queue_size=10)

    def on_message(this, ws, message):
        message_dict = json.loads(message)
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.pose.orientation.w = message_dict['values'][3]
        pose_stamped.pose.orientation.x = message_dict['values'][0]
        pose_stamped.pose.orientation.y = message_dict['values'][1]
        pose_stamped.pose.orientation.z = message_dict['values'][2]
        rospy.loginfo(f"Publishing pose: w: {pose_stamped.pose.orientation.w}, x: {pose_stamped.pose.orientation.x}, y: {pose_stamped.pose.orientation.y}, z: {pose_stamped.pose.orientation.z}")
        this.publisher.publish(pose_stamped)
        this.test.publish("test")

    def on_error(this, ws, error):
        rospy.loginfo("Error, failed to connect to websocket")

    def on_close(this, ws, close_code, reason):
        rospy.loginfo("Websocket connection closed")
        rospy.loginfo("Close code: ", close_code)
        rospy.loginfo("Reason: ", reason)

    def on_open(this, ws):
        rospy.loginfo("Websocket connected")

    def connect(this):
        rospy.loginfo("Connecting...")
        ws = websocket.WebSocketApp(this.url, on_open=this.on_open, on_message=this.on_message, on_error=this.on_error, on_close=this.on_close)
        ws.run_forever()

if __name__ == "__main__":
    rospy.init_node('android')
    dataClient = SensorDataClient("ws://Pixel-3-XL:8081/sensor/connect?type=android.sensor.rotation_vector")
    dataClient.connect()
