import matplotlib.pyplot as plt
def load(fname):
	x_list = []
	y_list = []
	t_list = []
	with(open(fname)) as f:
	    for line in f:
	        t,x,y = line.split()
	        x_list.append(x)
	        t_list.append(t)
	        y_list.append(y)
	return t_list, x_list, y_list

def plot(fname, axis="x"):
	t_list, x_list, y_list = load(fname)
	if axis == "x":
		plt.plot(t_list, x_list, label=fname.split("_")[-2:][0])
	if axis == "y":
		plt.plot(t_list, y_list, label=fname.split("_")[-2:][0])

import os 
def plot_files(ending_with="vel.log"):
	if ending_with == "vel.log":
		for ax in "x,y".split(','):
			for fname in os.listdir(os.curdir):
				if fname.endswith(ending_with):
					plot(fname, ax)
			plt.title("Velocities "+ax)
			plt.ylabel("mts/s")
			plt.xlabel("time")
			plt.legend()
			plt.show()
	else:
		for fname in os.listdir(os.curdir):
			if fname.endswith(ending_with):
				t_list, x_list, y_list = load(fname)
				plt.plot(x_list, y_list, label=fname.split("_")[-2:][0])
		plt.title("Poses")
		plt.ylabel("y position")
		plt.xlabel("x position")
		plt.legend()
		plt.show()

plot_files()
plot_files('poses.log')