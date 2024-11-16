# 微木 · Mumu

Mumu 是一个结构简单的虚拟机（抽象的计算模型），可以在资源有限的平台上创造相对灵活的计算环境。

## Mumu 计算模型

- Harvard 架构，随机存储（RAM）与只读存储（ROM）分离，每个存储单元均为 32 位
- 程序从 ROM 地址 0 处开始执行，每条指令占一个存储单元
- 256 个“通用寄存器” %0 \~ %255，直接映射到随机存储单元 0 \~ 255
- “堆栈指针寄存器” SP 为随机存储单元 M-1，其中 M 为随机存储单元总数

## Mumu 汇编语言

单个大写字母均表示寄存器（均指通用寄存器，不包括 SP，下同）。

参考：[Fibonacci 数列](./fib.mumu)、[天气蘑菇彩虹光效](./lights/3.mumu)。

### 算术
```
<op>  <A> <B> <C>     # A := B (op) C
<op>  <A> <B> <Imm8>  # A := B (op) Imm8
<op>  <A> <Imm16>     # A := A (op) Imm16
<op>  <A> <Imm16>     # A := A (op) (Imm16 << 16)
```
其中 `<op>` 为 `mov`/`add`/`sub`/`mul`/`and`/`or`/`xor`/`bic`/`lsr`/`asr`/`lsl` 之一。

### 访存
```
ld    <A> <B> <C>  # A := RAM[B + C]
st    <A> <B> <C>  # RAM[B + C] := A
ldc   <A> <B> <C>  # A := ROM[B + C]
ldcr  <A> <B> <C>  # A := ROM[PC + B + C]
```
另外，访存指令中所有的 `<C>` 也可以为 8 位立即值。

### 控制流
```
br<cond>  <A> <B> <Imm8>      # PC := PC + Imm8
br<cond>  <A> <Imm8B> <Imm8C> # PC := PC + Imm8C
br<cond>  <A> <B> <C>         # PC := PC + C
ba<cond>  <A> <B> <C>         # PC := C
baf       <Imm24>             # PC := Imm24
```
其中 `<cond>` 为 `al`/`eq`/`ne`/`gt`/`lt`/`ge`/`le`/`sgt`/`slt`/`sge`/`sle` 之一。

### 堆栈与子过程
```
push  <A> <Imm8>    # SP := SP - Imm8; RAM[SP..=SP+Imm8-1] = A..=A+Imm8-1
pop   <A> <Imm8>    # A..=A+Imm8-1 = RAM[SP..=SP+Imm8-1]; SP := SP + Imm8
call  <Imm24>       # push PC+1; baf Imm24
ret                 # pop PC
```

### 系统呼叫
```
sc  <Imm24>
```

## Mulin

希望以后能提供更加易于上手的语言。暂时还没有开始。
