from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer
from pybrain.structure.modules import TanhLayer

def to_binary(outn, maxn):
	s = [0] * maxn
	s[outn - 1] = 1
	return tuple(s) 

N = 2
net = buildNetwork(9, 5, N, hiddenclass=TanhLayer)
ds = SupervisedDataSet(9, N)
all_inputs = []
with open('samples.txt') as f:
	for line in f:
		inputs, output = line.split(';')
		inputs = map(int, inputs.split(','))
		ds.addSample(tuple(inputs), to_binary(int(output), N))
		all_inputs.append(inputs)
		
trainer = BackpropTrainer(net, dataset=ds, momentum=0.5, verbose=True, weightdecay=0.1)

for i in range(10):
	trainer.trainEpochs(20)

for inputs in all_inputs:
	print net.activate(inputs)
