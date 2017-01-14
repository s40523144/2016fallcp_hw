content = open("cpa.txt").read()
#print(content)
# 這是單行註解
result = content.splitlines()
# result 變數資料型別為 list
#print(result)
# 針對 list 資料, 可以利用 for 迴圈列出其數列成員
num = 1
for g in result:
    print(num, g)
    num = num + 1
    print()

'''
for line in content.splitlines():
result.append(list(line.split(",")))
'''