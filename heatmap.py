import math

# square root of the number of of physical pixels for each pixel in the heatmap
# smaller downsize factor means better precision, but slower algorithm
downsize_factor = 4

# draws a list of blobs on a heatmap
def draw(arr, blobs):
	for blob in blobs:
		for y in range(blob.left/downsize_factor, blob.right/downsize_factor):
			for x in range(blob.top/downsize_factor, blob.bottom/downsize_factor):
				arr[x,y] += 50 + math.sqrt(blob.area)/10

# cooldown, so activation of the heatmap slowly goes down without blobs
def cooldown(arr):
	arr *= .5
