c=0:10:80;
sita=0:0.2:1;
pis1=360;
pis2=340;
deltaspi=zeros(length(c),length(sita));
for i=1:length(c)
       for j=1:length(sita)
                deltaspi(i,j)=double(pis1-pis2-c(i)*sita(j));
       end
end
[c1,sita1] = meshgrid(c,sita);
surf(c,sita,deltaspi');