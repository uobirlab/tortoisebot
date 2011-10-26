package barc;

import org.ros.message.MessageListener;
import org.ros.message.geometry_msgs.Twist;
import org.ros.message.sensor_msgs.LaserScan;
import org.ros.message.turtlebot_node.TurtlebotSensorState;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.Node;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;

/**
 * 
 * @author hacker
 */
public class BraitenbergMover implements NodeMain {

	private static final double BACKING_OFF_SPEED = 0.00;
	private static final double LINEAR_FORWARD_SPEED = 0.1;
	private static final int TURN_SPEED_DEG = 45;
	private Node node;
	private Publisher<Twist> pub;

	public void main(NodeConfiguration nc) throws Exception {

		node = new DefaultNodeFactory().newNode(BraitenbergMover.class
				.getSimpleName(), nc);
		pub = node
				.newPublisher("turtlebot_node/cmd_vel", "geometry_msgs/Twist");

		node.newSubscriber("turtlebot_node/base_scan", "sensor_msgs/LaserScan",
				new MessageListener<LaserScan>() {

					@Override
					public void onNewMessage(LaserScan scan) {
						move(scan);
					}
				});
		node.newSubscriber("/turtlebot_node/sensor_state",
				"turtlebot_node/TurtlebotSensorState",
				new MessageListener<TurtlebotSensorState>() {

					@Override
					public void onNewMessage(TurtlebotSensorState arg0) {
						move(arg0);
					}
				});
	}

	protected void move(TurtlebotSensorState sensorState) {
		node.getLog().info("bump state=" + sensorState.bumps_wheeldrops);
		byte bump = sensorState.bumps_wheeldrops;
		Twist move = new Twist();

		if (bump == 1) {
			move.linear.x = -BACKING_OFF_SPEED;
			move.angular.z = Math.PI * TURN_SPEED_DEG / 180;
		} else if (bump == 2) {
			move.linear.x = -BACKING_OFF_SPEED;
			move.angular.z = Math.PI * TURN_SPEED_DEG / 180;
		} else if (bump == 3) {
			move.linear.x = -BACKING_OFF_SPEED;
			move.angular.z = Math.PI * TURN_SPEED_DEG / 180;
		} else if (bump>3){
			move.linear.x = 0.0;
			move.angular.z = 0.0;
		} else {
			move.linear.x = LINEAR_FORWARD_SPEED;
			move.angular.z = 0.0;
			
		}
		pub.publish(move);


	}

	private void move(LaserScan scan) {
		double left = 0.0, right = 0.0;

		// get space on each side
		for (int i = 0; i < scan.ranges.length; i++) {
			double range = (scan.ranges[i] == 0.0) ? scan.range_max
					: scan.ranges[i];
			if (i < scan.ranges.length / 2) {
				left += range;
			} else {
				right += range;
			}
		}

		// publish a twist command to move towards the area of greatest space
		Twist move = new Twist();
		move.linear.x = 0.1;
		move.angular.z = (right - left) / (scan.ranges.length) * 0.5;
		pub.publish(move);
	}

	public void shutdown() {
		node.shutdown();
	}
}