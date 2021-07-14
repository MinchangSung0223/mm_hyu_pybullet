import matplotlib.pyplot as plt
import numpy as np

def drawplot(t,v,title,xlabel,ylabel):
	fig, ax = plt.subplots()
	ax.plot(t, v)
	ax.set(xlabel=xlabel, ylabel=ylabel,title=title)
	ax.grid()

	fig.savefig("test.png")
	plt.show()

