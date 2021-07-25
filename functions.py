import matplotlib.pyplot as plt
import numpy as np

def drawplot(t,v,title,xlabel,ylabel):
	fig, ax = plt.subplots()
	ax.plot(t, v)
	ax.set(xlabel=xlabel, ylabel=ylabel,title=title)
	ax.grid()

	fig.savefig("test.png")
	plt.show()
def drawqplot(t,q,dq,title,xlabel,ylabel,ROBOT_DOF):

	fig, axs = plt.subplots(ROBOT_DOF)
	for j in range(ROBOT_DOF):
		axs[j].plot(t,q[:,j],t,dq[:,j])
		axs[j].set(xlabel=xlabel, ylabel="q"+str(j),title=title)
		axs[j].set_xlim([0,max(t)])
		axs[j].set_ylim([-np.pi,np.pi])
		axs[j].grid()
	plt.show()


	# 	plt.subplot(ROBOT_DOF,1,j+1)
	# 	q_ = q[:,j]
	# 	print(t)
	# 	ax.plot(t, np.array(q_))
	# 	ax.set(xlim=(0,t[len(t)-1]),ylim=(min(q_),max(q_)))
	# ax.set(xlabel=xlabel, ylabel=ylabel,title=title)
	# ax.grid()
	# fig.savefig("test.png")
	# plt.show()


