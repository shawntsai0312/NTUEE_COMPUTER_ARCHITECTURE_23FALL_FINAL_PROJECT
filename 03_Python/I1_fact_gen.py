def fact(n):
    if n < 1:
        return 1
    else:
        return n*fact(n-1)

if __name__ == '__main__':
    # Modify your test pattern here
    n = 5
        
    with open('../00_TB/Pattern/I1/mem_D.dat', 'w') as f_data:
        f_data.write(f"{n:08x}\n")

    with open('../00_TB/Pattern/I1/golden.dat', 'w') as f_ans:
        f_ans.write('{:0>8x}\n'.format(n))
        f_ans.write('{:0>8x}\n'.format(fact(n)))