#!/usr/bin/env python

import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
import sensor_msgs
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
import sensor_msgs.point_cloud2 as pcl2
import pcl
import struct
import math
import numpy as np

class LidarRGB:

	def __init__( self ):
		self._imageInputName = rospy.resolve_name( 'image' )
		print( self._imageInputName )

		self._velodyneOutputName = rospy.resolve_name( 'velodyne_rgb_points' )
		print( self._velodyneOutputName )

		self._cameraName = rospy.resolve_name( 'camera' )
		print( self._cameraName )

		self._velodyneName = rospy.resolve_name( 'velodyne' )
		print( self._velodyneName )

		self._imageInput = rospy.Subscriber( self._imageInputName, Image, callback = self.imageCallback, queue_size = 1 )
		self._camera = rospy.Subscriber( self._cameraName, CameraInfo, callback = self.cameraCallback, queue_size = 1 )
		self._velodyne = rospy.Subscriber( self._velodyneName, PointCloud2, callback = self.velodyneCallback, queue_size = 1 )

		self._velodyneOutput = rospy.Publisher( self._velodyneOutputName, PointCloud2, queue_size = 1 )

		self._bridge = cv_bridge.CvBridge()
		self._cameraModel = PinholeCameraModel()
		self._tf = tf.TransformListener()

	def imageCallback( self, data ):
		print( 'Received an image!' )

		self._image = {}
		try:
			self._image = self._bridge.imgmsg_to_cv2( data, 'bgr8' )
		except cv_bridge.CvBridgeError as e:
			rospy.logerr( '[lidar_rgb] Failed to convert image' )
			rospy.logerr( '[lidar_rgb] ' + e )
			print( e )
			return

		( translation, rotation ) = self._tf.lookupTransform( 'world', 'camera', rospy.Time( 0 ) )

		# print( translation )
		translation = translation + ( 1,  )
		# print( translation )
		self._Rq = tf.transformations.quaternion_matrix( rotation )
		# print( Rq )
		self._Rq[ :, 3 ] = translation

	def cameraCallback( self, data ):
		print( 'Received camera info' )
		self._cameraModel.fromCameraInfo( data )

	def velodyneCallback( self, data ):
		print( 'Received velodyne point cloud' )

		# add r, g, b fields
		fields = []
		fields.append( PointField( 'x', 0, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'y', 4, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'z', 8, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'intensity', 12, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'r', 16, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'g', 20, PointField.FLOAT32, 1 ) )
		fields.append( PointField( 'b', 24, PointField.FLOAT32, 1 ) )

		if hasattr( self, '_Rq' ) and hasattr( self, '_image' ):
			height, width, channels = self._image.shape

			points = []
			for dataPoint in pcl2.read_points( data ):
				point = [ dataPoint[ 0 ], dataPoint[ 1 ], dataPoint[ 2 ], 1.0 ]
				intensity = dataPoint[ 3 ]
				
				rotatedPoint = self._Rq.dot( point )

				# only process points in front of the camera
				if rotatedPoint[ 2 ] < 0:
					continue

				# project point into image coordinates
				uv = self._cameraModel.project3dToPixel( rotatedPoint )
				
				# add point only if it's within the image bounds
				if uv[ 0 ] >= 0 and int( uv[ 0 ] )  < height and uv[ 1 ] >= 0 and int( uv[ 1 ] ) < width:
					[ b, g, r ] = self._image[ int( uv[ 1 ] ), int( uv[ 0 ] ) ]
					points.append( [ point[ 0 ], point[ 1 ], point[ 2 ], intensity, r / 255.0, g / 255.0, b / 255.0 ] )

			print( 'Transmitting lidar_rgb' )
			newMessage = pcl2.create_cloud( data.header, fields, points )
			self._velodyneOutput.publish( newMessage )

if __name__ == '__main__':
	try:
		rospy.init_node( 'lidar_rgb' )
		lidarRGB = LidarRGB()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
