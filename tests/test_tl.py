import tl
import numpy as np

def test_TL_zero_case():
	#test 0 case
	try:
		bot1 = tl.TL(0,0)

	except Exception as e:
		print(e)

def test_TL_book_case():
	#test book example x, y = 10.0, 8.0
	try:
		bot2 = tl.TL(15.0, 10.0)
		theta_1, theta_2= bot2.IK(10.0, 8.0)
		#print (theta_1, theta_2)
		#returns t1 = 1.39408671883 t2 = -2.13727804092

	except Exception as e:
		print(e)

def test_TL_zero_link1_case():
	#test length 0 on link 1
	try:
		bot3 = tl.TL(15.0, 10.0)

	except Exception as e:
		print(e)

def test_TL_zero_link2_case():
	#tst length 0 on link 2
	try:
		bot4 = tl.TL(15.0, 10.0)

	except Exception as e:
		print(e)

def test_TL_negative_case():
	#test negative angles
	try:
		bot5 = tl.TL(15.0, 10.0)

	except Exception as e:
		print(e)
