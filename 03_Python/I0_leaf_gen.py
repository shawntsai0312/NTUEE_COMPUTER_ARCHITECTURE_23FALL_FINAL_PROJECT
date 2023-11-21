def leaf(a,b,c,d):
    f = (a+b) - (c+d)
    return f
def toHex(x):
    if x >= 0:
        return x
    else:
        return 2**32+x

if __name__ == '__main__':
    # Modify your test pattern here
    a = 7
    b = 4
    c = 1
    d = 4

    with open('../00_TB/Pattern/I0/mem_D.dat', 'w') as f_data:
        f_data.write(f"{a:08x}\n")
        f_data.write(f"{b:08x}\n")
        f_data.write(f"{c:08x}\n")
        f_data.write(f"{d:08x}\n")

    with open('../00_TB/Pattern/I0/golden.dat', 'w') as f_ans:
        f_ans.write('{:0>8x}\n'.format(toHex(a)))
        f_ans.write('{:0>8x}\n'.format(toHex(b)))
        f_ans.write('{:0>8x}\n'.format(toHex(c)))
        f_ans.write('{:0>8x}\n'.format(toHex(d)))
        f_ans.write('{:0>8x}\n'.format(toHex(leaf(a,b,c,d))))