import math

downsize_factor = 4

def draw(arr, blobs):
	for blob in blobs:
		for y in range(blob.left/downsize_factor, blob.right/downsize_factor):
			for x in range(blob.top/downsize_factor, blob.bottom/downsize_factor):
				arr[x,y] += 50 + math.sqrt(blob.area)/10

def cooldown(arr):
	arr *= .5
