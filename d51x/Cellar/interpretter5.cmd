//valset(1,0)
valmath(2,minutesperday%120)
valset(3,30)
if(valdes[0]==1&&valdes[1]>60)
valset(0,1)
else
if(valdes[0]==0&&valdes[1]<50)
valset(0,0)
endif
valset(0,0)
endif
if(valget(2)>=0&&valget(2)<5)
valset(1,1)
else
valset(1,0)
endif
if(valget(0)==1||valget(1)==1)
gpioset(220,1)
printw(Откачиваем)
else
gpioset(220,0)
printw(Не откачиваем)
endif
