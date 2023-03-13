In four folders is the code for all the figures in the report.
====================================
'Practice_3' is the code for Section 1.2
	'IC_Nominal.m' is main
___________________________________________________
'Robust Interpolation control' is for Section 1.3
	'Practice4.m' is main
___________________________________________________
'Improved IC with Minkowski' is for Section 1.4
	'Improved_IC_Tracking.m' is the one without frozen target for simple trajectory
	'Improved_IC_Tracking2.m' is the one with frozen target for simple trajectory
	'Improved_IC_Tracking3.m' is the one with frozen target for complecated trajectory
________________________________
'MPCTracking' is for Section 2
	'MPC_Tracking.m' is the controller written by myself. By switching between 'ExpandSystem' or 'ExpandSystem3' function used in the code, one can get results of MPC1 and MPC2 in my report respectivly.
	'MPC_Tracking2.m' is the controller using MPT3, this controller computs slower than the one written by myself. I suppose the reason is because this way needs to create a new system each time step (because the system is time varying).
	'test.m' is the code for example 3.1 on book.