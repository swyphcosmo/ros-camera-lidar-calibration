#!/usr/bin/env python

import cv2
import cv_bridge
from image_geometry import PinholeCameraModel
import tf
import sensor_msgs
import rospy
import sys
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
import pcl
import struct
import math
import numpy as np

class LidarImage:

	def __init__( self ):
		self._imageInputName = rospy.resolve_name( 'image' )
		print( self._imageInputName )

		self._imageOutputName = rospy.resolve_name( 'image_lidar' )
		print( self._imageOutputName )

		self._cameraName = rospy.resolve_name( 'camera' )
		print( self._cameraName )

		self._velodyneName = rospy.resolve_name( 'velodyne' )
		print( self._velodyneName )

		self._imageInput = rospy.Subscriber( self._imageInputName, Image, callback = self.imageCallback, queue_size = 1 )
		self._camera = rospy.Subscriber( self._cameraName, CameraInfo, callback = self.cameraCallback, queue_size = 1 )
		self._velodyne = rospy.Subscriber( self._velodyneName, PointCloud2, callback = self.velodyneCallback, queue_size = 1 )

		self._imageOutput = rospy.Publisher( self._imageOutputName, Image, queue_size = 1 )

		self._bridge = cv_bridge.CvBridge()
		self._cameraModel = PinholeCameraModel()
		self._tf = tf.TransformListener()

	def imageCallback( self, data ):
		print( 'Received an image!' )

		cvImage = {}
		try:
			cvImage = self._bridge.imgmsg_to_cv2( data, 'bgr8' )
		except cv_bridge.CvBridgeError as e:
			rospy.logerr( '[lidar_image] Failed to convert image' )
			rospy.logerr( '[lidar_image] ' + e )
			print( e )
			return

		( translation, rotation ) = self._tf.lookupTransform( 'world', 'velodyne', rospy.Time( 0 ) )

		# print( translation )
		translation = translation + ( 1,  )
		# print( translation )
		Rq = tf.transformations.quaternion_matrix( rotation )
		# print( Rq )
		Rq[ :, 3 ] = translation

		if self._velodyneData:

			for i in range( 0, len( self._velodyneData ) - 1 ):
				try:
					point = self._velodyneData[ i ][ :3 ] + ( 1, )
					if math.sqrt( np.sum( np.array( point[ :3 ] ) ** 2 ) ) > 4.0:
						continue

					intensity = self._velodyneData[ i ][ 3 ]
				except IndexError:
					print( i )
					print( self._velodyneData[ i ] )
					break

				if intensity < 0.001:
					continue

				# point = ( 3.001, 0.873, -0.273, 1 )
				# print( point )
				rotatedPoint = Rq.dot( point )
				if rotatedPoint[ 2 ] < 0:
					continue

				# print( rotatedPoint )

				uv = self._cameraModel.project3dToPixel( rotatedPoint )

				# print( uv )
				# print( cvImage.shape )

				if uv[ 0 ] >= 0 and uv[ 0 ] <= data.width and uv[ 1 ] >= 0 and uv[ 1 ] <= data.height:
					# print( 'on image' )
					intensityInt = int( intensity )
					intensityInt = ( 255 - intensityInt ) * 255 * 255 
					cv2.circle( cvImage, ( int( uv[ 0 ] ), int( uv[ 1 ] ) ), 2, cv2.cv.Scalar( intensityInt % 255, ( intensityInt / 255 ) % 255, ( intensityInt / 255 / 255 ) ) )

		try:
			self._imageOutput.publish( self._bridge.cv2_to_imgmsg( cvImage, 'bgr8' ) )
		except cv_bridge.CvBridgeError as e:
			rospy.logerr( '[lidar_image] Failed to convert image to message' )
			rospy.logerr( '[lidar_image] ' + e )
			print( e )
			return

	def cameraCallback( self, data ):
		print( 'Received camera info' )
		# print( data )
		self._cameraModel.fromCameraInfo( data )
		# self._cameraInfo = data

	def velodyneCallback( self, data ):
		print( 'Received velodyne point cloud' )
		# self._velodyneData = data
		# for field in self._velodyneData.fields:
			# print( field )
		# print( type( data.data ) )
		# print( data.data[ 0 ] )
		formatString = 'ffff'
		if data.is_bigendian:
			formatString = '>' + formatString
		else:
			formatString = '<' + formatString

		points = []
		# points.append( struct.unpack( formatString, data.data[ :16 ] ) )
		# points.append( struct.unpack( formatString, data.data[ 16:32 ] ) )
		# print( points )

		for index in range( 0, len( data.data ), 16 ):
			points.append( struct.unpack( formatString, data.data[ index:index + 16 ] ) )

		print( len( points ) )
		self._velodyneData = points

if __name__ == '__main__':
	try:
		rospy.init_node( 'lidar_image' )
		lidarimage = LidarImage()
		rospy.spin()

	except rospy.ROSInterruptException:
		pass
