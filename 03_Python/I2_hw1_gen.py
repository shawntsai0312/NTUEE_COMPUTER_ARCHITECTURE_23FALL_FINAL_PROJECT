import math

def hw1(n):
    if n >= 2:
        y = 5*hw1(math.floor(n/2))+6*n+4
        return y
    else:
        return 2

if __name__ == '__main__':
    # Modify your test pattern here
    n = 10
        
    with open('../00_TB/Pattern/I2/mem_D.dat', 'w') as f_data:
        f_data.write(f"{n:08x}\n")


    with open('../00_TB/Pattern/I2/golden.dat', 'w') as f_ans:
        f_ans.write('{:0>8x}\n'.format(n))
        f_ans.write('{:0>8x}\n'.format(hw1(n)))