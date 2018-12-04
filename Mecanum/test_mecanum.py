import mecanum


#test mecanum initialization
def test_MECANUM_initilization_case():
	try:
		bot= mecanum.MECANUM(1, 1, 1)

	except Exception as e:
		print(e)

#test wheel radius = 0
def test_MECANUM_zero_case_rad():
	#test 0 case
	try:
		bot= mecanum.MECANUM(0, 1, 1)

	except Exception as e:
		print(e)

#test left right wheels
def test_MECANUM_zero_case_horizontal():
	#test 0 case
	try:
		bot= mecanum.MECANUM(1, 0, 1)

	except Exception as e:
		print(e)

#test front back wheels
def test_MECANUM_zero_case_vertical():
	#test 0 case
	try:
		bot= mecanum.MECANUM(1, 1, 0)

	except Exception as e:
		print(e)


def test_MECANUM_FK_case():
	#test book example if make is possible
	try:
		bot= mecanum.MECANUM(1, 1, 0)

	except Exception as e:
		print(e)

def test_MECANUM_IK_case():
	#test book example if make is possible
	try:
		bot= mecanum.MECANUM(1, 1, 0)

	except Exception as e:
		print(e)


#def test_MECANUM_global_FK_case():
	#test length 0 on link 1

#def test_MECANUM_global_IK_case():
	#tst length 0 on link 2


#def test_MECANUM_negative_case():
	#test negative angles


