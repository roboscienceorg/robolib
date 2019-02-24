from context import ptl


#assert

def test_PTL_zero_case():
	#test 0 case
	try:
		bot1 = ptl.PTL(0,0,0)

	except Exception as e:
		print(e)


def test_PTL_book_case():
	#test book example if make is possible
	try:
		bot2 = ptl.PTL(8,5,10)
		ang_list = bot2.IK(0,4)
		print(ang_list)

	except Exception as e:
		print(e)

def test_PTL_book_case_dos():
	#test book example if motion is possible
	try:
		bot2 = ptl.PTL(8,5,10)

		x = np.arange(-3, 3, 0.2)
		y = 0.1*x + 10

		for i in range(1,30):
			ang_list = bot2.IK(x[i],y[i])
			print(ang_list)

	except Exception as e:
		print(e)


def test_PTL_short_link1_case():
	#test length 0 on link 1
	try:
		bot3 = ptl.PTL(8,0,10)

	except Exception as e:
		print(e)

def test_PTL_short_link2_case():
	#tst length 0 on link 2
	try:
		bot4 = ptl.PTL(8,5,0)

	except Exception as e:
		print(e)

def test_PTL_negative_case():
	#test negative angles
	try:
		bot5 = ptl.PTL(8,5,10)

	except Exception as e:
		print(e)

