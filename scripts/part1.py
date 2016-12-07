import cv2
from camera_calibration.calibrator import MonoCalibrator, ChessboardInfo

numImages = 30

images = [ cv2.imread( '../Images/frame{:04d}.jpg'.format( i ) ) for i in range( numImages ) ]

board = ChessboardInfo()
board.n_cols = 7
board.n_rows = 5
board.dim = 0.050

mc = MonoCalibrator( [ board ], cv2.CALIB_FIX_K3 )
mc.cal( images )
print( mc.as_message() )

mc.do_save()


