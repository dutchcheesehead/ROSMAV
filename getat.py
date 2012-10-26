def getat(img, d):
    return getatraw(img, d.x, d.y)

def getatraw(img, x, y):
    return (ord(img.data[x * 3 + y * img.step]),
        ord(img.data[x * 3 + y * img.step + 1]),
        ord(img.data[x * 3 + y * img.step + 2]))
