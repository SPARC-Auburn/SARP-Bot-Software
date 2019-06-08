import os

nodes = os.popen("rosnode list").readlines()
for i in range(len(nodes)):
	nodes[i] = nodes[i].replace("\n", "")
for node in nodes:
	if node != "roscore":
		os.system("rosnode kill " + node)
