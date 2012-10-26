def getat(d):
    x, y = d.x, d.y
    return (ord(img.data[x * 3 + y * img.step]),
        ord(img.data[x * 3 + y * img.step + 1]),
        ord(img.data[x * 3 + y * img.step + 2]))

