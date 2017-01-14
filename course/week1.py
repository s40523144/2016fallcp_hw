# 先定義函式
# 直接列印字串的函式
def test():
    print("test")

# 傳回字串的函式
def test1():
    return "test"
    
# 然後再呼叫
test()

# s 將接受 test1() 函式的傳回資料
s = test1()
print(s)

# print() 函式用法
##################################################
print("Hello World!")
# 利用 help() 列出 print 函式的用法說明
print(help(print))
# 利用 help() 列出 int 函式的用法說明
print(help(int))
# 導入 keyword 模組
##################################################
import keyword
# 列出 keyword 模組的 kwlist
print(keyword.kwlist)

# Python3 的變數與資料型別
##################################################
# str(), int(), id(), float()
'''
Python has 5 standard data types namely.

a) Numbers
b) String
c)  List
d) Tuple
e) Dictionary
f)  Boolean – In Python True and False  are boolean literals.  
'''
# 有關 input() 的用法
##################################################

# 導入 math 模組
##################################################

import math
print(math.pi)

'''
# http://thepythonguru.com/python-numbers/
Python 3 support 3 different numerical types.

int – for integer values like 45 .
float – for floating point values like 2.3 .
complex – for complex numbers like 3+2j .
'''
# 字串處理
##################################################

# List 數列
##################################################

# dictionary 字典
##################################################

# tuples 元組
##################################################

# control statement
##################################################

# 函式
##################################################

# 迴圈
##################################################

# File IO
##################################################

# 數學函式
##################################################

# 物件與類別
##################################################

# Operator Overloading
#http://thepythonguru.com/python-operator-overloading/
##################################################

# 繼承與多型
# http://thepythonguru.com/python-inheritance-and-polymorphism/
##################################################

# 例外處理
##################################################

# 模組
##################################################

# http://thepythonguru.com/running-python-programs/
# 設定 /etc/init/brython-server.conf
# 設定 /etc/nginx/sites-available/default
# redis 自動啟動
# 只要在 8888 虛擬機器加入 IPV4 的設定
# 還有讓 8888 納入 IPV4 DNS 設定
# 就可以正常與 Github API 主機透過 IPV4 交換 token 了
# 讓協同者也可以修改, 並且提交推送
# 使用井字號開頭的這一行, 為註解, Python 解譯器不會執行
'''
以3個單引號, 或3個雙引號前後圈住的內容, 為多行註解, Python 解譯器也不會執行
https://blog.openshift.com/enabling-redis-for-your-app/
'''
# 選擇一個變數, 名稱定為 repeat, 且將整數 10 與此變數對應
'''
repeat = 10
for i in range(repeat):
    print("hello world!")
    
# http://www.learnpython.org/
# change this code
mystring = "hello"
myfloat = 10.0
myint = 20

# testing code
if mystring == "hello":
    print("String: %s" % mystring)
if isinstance(myfloat, float) and myfloat == 10.0:
    print("Float: %d" % myfloat)
if isinstance(myint, int) and myint == 20:
    print("Integer: %d" % myint)
    
numbers = []
strings = []
names = ["John", "Eric", "Jessica"]

# write your code here
second_name = None


# this code should write out the filled arrays and the second name in the names list (Eric).
print(numbers)
print(strings)
print("The second name on the names list is %s" % second_name)

x = object()
y = object()

# TODO: change this code
x_list = [x]
y_list = [y]
big_list = []

print("x_list contains %d objects" % len(x_list))
print("y_list contains %d objects" % len(y_list))
print("big_list contains %d objects" % len(big_list))

# testing code
if x_list.count(x) == 10 and y_list.count(y) == 10:
    print("Almost there...")
if big_list.count(x) == 10 and big_list.count(y) == 10:
    print("Great!")
'''