from pybrain.tools.shortcuts import buildNetwork
from pybrain.datasets import SupervisedDataSet
from pybrain.supervised.trainers import BackpropTrainer

def to_binary(outn, maxn):
	s = [0] * maxn
	s[outn - 1] = 1
	return s 

N = 3
net = buildNetwork(9, 5, N)
ds = SupervisedDataSet(9, N)
with open('samples.txt') as f:
	for line in f:
		inputs, output = line.split(';')
		inputs = map(int, inputs.split(','))
		ds.addSample(inputs, to_binary(int(output), N))
		
trainer = BackpropTrainer(net, dataset=ds, momentum=0.1, verbose=True, weightdecay=0.01)

for i in range(20):
	trainer.trainEpochs(1)
