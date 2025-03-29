import qrcode

links = [ 
("MarbleAccelerator", "https://github.com/fabianmink/MarbleAccelerator"),
]


for link in links:
    name = link[0]
    url = link[1]
    print("name: " + name + " URL: " + url)
    filename = name + ".png"

    qr = qrcode.QRCode(
        version=1,
        box_size=10,
        border=0)
    qr.add_data(url)
    qr.make(fit=True)
    img = qr.make_image(fill='black', back_color='white')
    img.save(filename)