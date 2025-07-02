#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

// Initialize BNO055 IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ROS Node Handle
ros::NodeHandle nh;

// ROS Publishers
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
ros::Publisher imu_pub("/imu/data", &imu_msg);
ros::Publisher mag_pub("/imu/mag", &mag_msg);

void setup() {
  // Initialize Serial for debugging and ROS
  Serial.begin(115200);
  nh.initNode();
  nh.advertise(imu_pub);
  nh.advertise(mag_pub);

  // Initialize BNO055 IMU
  if (!bno.begin()) {
    Serial.println("BNO055 not detected. Check wiring!");
    while (1); // Halt if IMU is not detected
  }

  // Use external crystal for better accuracy
  bno.setExtCrystalUse(true);

  Serial.println("BNO055 initialized successfully.");
}


void loop() {
  // Set frame_id and timestamp
  imu_msg.header.frame_id = "imu_link"; // Define the IMU frame ID
  imu_msg.header.stamp = nh.now();     // Set the ROS timestamp

  mag_msg.header.frame_id = "imu_link"; // Use the same frame for magnetometer
  mag_msg.header.stamp = nh.now();      // Set the ROS timestamp

  // Get Orientation (Quaternion)
  imu::Quaternion quat = bno.getQuat();

  // Populate orientation in IMU message
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();

  // Get Angular Velocity (Gyroscope)
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();

  // Get Linear Acceleration (Accelerometer)
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu_msg.linear_acceleration.x = accel.x();
  imu_msg.linear_acceleration.y = accel.y();
  imu_msg.linear_acceleration.z = accel.z();

  // Publish the IMU message
  imu_pub.publish(&imu_msg);

  // Get Magnetic Field Data
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  mag_msg.magnetic_field.x = mag.x();
  mag_msg.magnetic_field.y = mag.y();
  mag_msg.magnetic_field.z = mag.z();

  // Publish the Magnetic Field message
  mag_pub.publish(&mag_msg);

  // Debug Output for Calibration and Data
  uint8_t sys, gyro_cal, accel_cal, mag_cal;
  bno.getCalibration(&sys, &gyro_cal, &accel_cal, &mag_cal);
  Serial.print("Calibration - SYS: ");
  Serial.print(sys);
  Serial.print(", GYRO: ");
  Serial.print(gyro_cal);
  Serial.print(", ACCEL: ");
  Serial.print(accel_cal);
  Serial.print(", MAG: ");
  Serial.println(mag_cal);

  Serial.print("Orientation (Quaternion): ");
  Serial.print(quat.x());
  Serial.print(", ");
  Serial.print(quat.y());
  Serial.print(", ");
  Serial.print(quat.z());
  Serial.print(", ");
  Serial.println(quat.w());

  Serial.print("Magnetic Field: ");
  Serial.print(mag.x());
  Serial.print(", ");
  Serial.print(mag.y());
  Serial.print(", ");
  Serial.println(mag.z());

  nh.spinOnce();
  delay(10);
}
