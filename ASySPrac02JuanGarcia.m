%% P02: Se�ales en Tiempo Cont�nuo
% 
% <<UPIITA.PNG>>
%
% *UNIDAD PROFESIONAL INTERDISCIPLINARIA EN INGENIER�A Y TECNOLOG�AS AVANZADAS* 
% 
% *An�lisis de Se�ales y Sistemas*
%
% Autor: Garc�a Sandoval Juan Jes�s 2MV1
%
% Profesor: Dr Rafael Mart�nez Mart�nez
%
%% Objetivos
% Como objetivos principales de la pr�ctica se pretende implementar la manipulaci�n b�sica de MATLAB, 
% as� como la graficaci�n de se�ales reales y complejas continuas, la transformaci�n de se�ales continuas 
% (escalamientos y traslaciones) y el c�lculo de energ�a y potencia de se�ales continuas
%% Introducci�n
%
% En la presente pr�ctica se desarrollar� una p�gina web, en lenguaje html utilizando MATLAB.
% 
% MATLAB es un software matem�tico desarrollado por MathWorks y su nombre proviene de
% la Abreviaci�n de MATrix LABoratory. 
%
% MATLAB surgi� a partir que Cleve Moler, co-fundador de MathWorks, matem�tico estadounidense y 
% programador de ordenador que se especializa en an�lisis num�rico, desarroll� 
% un paquete de inform�tica num�rico, para dar su alumnado en la Universidad de 
% Nuevo M�xico, surgiendo en 1984 la primera versi�n estable del software.
%
% MATLAB implementa un lenguaje de programaci�n propio (lenguaje M) para
% desarrollar aplicaciones y realizar c�lculos de an�lisis num�rico.
%
% MATLAB es un software muy �til para la resoluci�n de problemas
% matem�ticos, sin embargo existen otras alternativas muy similares a
% MATLAB, por ejemplo <https://www.gnu.org/software/octave/ Octave> que se
% considera como la versi�n libre de MATLAB,
% <https://www.maplesoft.com/products/maple/ Maple> que es una alternativa
% muy empleada, y por parte de Wolfram est�
% <https://www.wolfram.com/mathematica/ Mathematica> y
% <https://www.wolframalpha.com/ WolframAlpha> esta �ltima es una
% plataforma en linea que carece de c�digo riguroso por lo que es muy
% senillo de usar, pero tambi�n carece de caracter�sticas como MATLAB o
% Mathematica.
%
%% 1
%
% Se cre� una funci�n llamada _fun1_ la cual recibe dos parametros $\omega$ y $a$ 
% para entonces regesar la evaluaci�n $F(\omega)=\frac{a}{a^2+\omega^2}$
% como se muestra a continuaci�n:
%
% Proponrmos $a$ y $\omega$ con los siguientes valores:
a=1;
w=2;
F(w)=fun1(a,w);
%%
% El c�digo de la funci�n _fun1_ se encuentra en el ap�ndice 1
%% 2
%
% Se crear� una funci�n que nos permita graficar con un estilo predefinido.
% En este caso se graficar� el ejemplo del inciso anterior
% $F(\omega)=\frac{a}{a^2+\omega^2}$. La funci�n se invoca de la siguiente manera:  _Figura(valores en x,
% valores en y, 't�tulo del gr�fico')_

w=-2:0.001:2;
a=1;
f=@(w) a./(a^2+w.^2);
Figura(w,f(w),'F(w)')
%%
% El c�digo de la funci�n _Figura_ se encuentra en el ap�ndice 2.
%% 3
% 
% Se crear� una funci�n que gr�fique funciones de $f:R\rightarrow R^2$ con
% un estilo predefinido. Para este caso se graficar� el espectro de
% magnitud y de fase de la transformada de Fourier de la funci�n
% $f(t)=e^{-2t}u(t)$.
%
% Tenemos entonces que
%
% $$f(t)=e^{-2t}u(t)\Longleftrightarrow X(\omega)=\frac{1}{2+j\omega}$$
% 
% pasando a su forma polar: 
%
% $$X(\omega)=\frac{1}{\sqrt{4+\omega^2}}e^{-jtan^{-1}(\frac{\omega}{2})}$$
% 
% por lo tanto: 
%
% $|X(\omega)|=\frac{1}{\sqrt{4+\omega^2}}\:\:\:y\:\:\:\angle X(\omega)=-tan^{-1}(\frac{\omega}{2})$
%
% Entonces podemos emplear la funci�n para graficar el espectro de fase y
% de magnitud, donde 
clear all
w=-10:0.01:10;
f=@(w) 1./sqrt(4+w.^2);
g=@(w) (-1).*atan(w./2);
Figura2(w,f(w),g(w))
%%
% 
% De igual manera podemos graficar la funci�n original $f(t)=e^{-2t}u(t)$
% con la funci�n hecha en el punto 2:
clear all
t=-10:0.01:10;
f=@(t) exp(-2.*t);
Figura(t,f(t),'f(t)')
%%
% El c�digo de la funci�n _Figura_ y el c�digo de la funci�n _Figura2_ se encuentra en 
% el ap�ndice 2 y en el ap�ndice 3 respectivamente.
%
%% 4
%
% *M1.1 Inline Functions*
%
% definimos la funci�n $f(t)=e^{-t}cos(2\pi t)$:
clear all
f=@(t) exp(-t).*cos(2*pi*t);
%%
% evaluamos la funci�n en cero:
f(0)
%%
% tambi�n podemos evaluar la funci�n en un vector definido:
t=(-2:2);
f(t)
%%
% Finalmente podemos graficar $f(t)$ contra $t$:
close all
plot(t,f(t));
xlabel('t'); ylabel('f(t)'); grid;
%%
% Lo que nos arroja una gr�fica poco entendible, asi que daremos m�s
% elementos al vector $t$ y graficaremos de nuevo:
t=-2:0.01:2;
close all
plot (t,f(t))
xlabel('t'); ylabel('f(t)'); grid;
%%
% en esta gr�fica si podemos analizar la se�al con claridad.
%
%
%%
% *M1.2 Relational Operators and the Unit Step Function*
%
% Definimos la funci�n $u(t)$
u=@(t) (t>=0&t>=0);
%%
% posteriormente construimos un vector $t$ con valores enteros y graficamos: 
t=(-2:2);
close all
plot(t,u(t))
xlabel('t'); ylabel('u(t)'); grid;
%%
% como podemos ver, la gr�fica no se parece en lo absoluto al escal�n
% unitario, por lo que le damos m�s elementos al vector $t$:
t=(-2:0.01:2);
close all
plot (t,u(t));
xlabel('t'); ylabel('u(t)');
axis([-2 2 -0.1 1.1])
%% 
% podemos definir una funci�n similar a una resta de escalones, por ejemplo
% $u(t)-u(t-1)$ de la siguiente forma:
p = @(t)(t>=0 & t<1);
t = (-1:0.01:2); plot(t,p(t))
xlabel('t'); ylabel('p(t) = u(t)-u(t-1)');
axis ([-1 2 -.1 1.1]);
%%
% De esta manera podmos construir escalones unitarios y diferencias entre ellos.
%%
%
% *M1.3 Visualizing Operations on the Independent Variable*
%
% Construimos la funci�n $g(t)=f(t)u(t)=f(t)=e^{-t}cos(2\pi t)u(t)$ de la
% siguiente forma:
g=@(t) exp(-t).*cos(2*pi*t).*(t>=0);
%%
% construimos un vector _t_ y graficamos $g(2t+1)$:
t = (-2:0.01:2);
plot(t,g(2*t+1)); xlabel('t'); ylabel('g(2t+1)'); grid;
%%
% tambien podemos aplicar una inversi�n horizontal multiplicando el
% argumento de la funci�n por un menos $g(-t+1)$:
plot(t,g(-t+1)); xlabel('t'); ylabel('g(-t+1)'); grid;
%%
% como parte final del inciso, pomos realizar una suma de funciones
% $h(t)=g(2t+1)+g(-t+1)$ de la siguiente manera:
plot(t,g(2*t+1) +g(-t+1)); xlabel('t'); ylabel('h(t)'); grid;
%%
%
% *M1.4 Numerical Integration and Estimating Signal Energy*
%
% Podemos calcular la energ�a de una se�al mediante la ecuaci�n $E_x=\int_{-\infty}^\infty |x(t)|^2dt$
% 
% Para llevar a cabo esto en MATLAB definimos la funci�n $x(t)$ y construimos un vector _t_:
clear all
x=@(t)exp(-t).*((t>=0)&(t<1));
t=(0:0.01:1);
%%
% posteriormente utilizamos es comando de _sum_ para calcular la energ�a:
Ex=sum(x(t).^2*0.01)
%%
% resultado que no es perfecto ya que puede tener al rededor de 1% de error
% relativo. Sin embargo para tener un resultado mucho m�s exacto (un error relativo de 0.0026%) podemos
% definir primero la funci�n al cuadrado, es decir:
x2=@(t) exp(-2*t);
Ex = integral(x2,0,1)
%%
% Otro ejemplo m�s complejo podr�a ser el siguiente: 
%
% $$E_x=\int_{-\infty}^\infty e^{-2t}cos^2(2\pi t)dt$$
%
g2 =@(t) exp(-2.*t).*(cos(2.*pi.*t).^2).*(t>=0);
t = (0:0.01:100);
Eg = integral(g2, 0,100)
%% 5
%
% Para emplear de manera pr�ctica lo visto en el inciso anterior, se
% resolver� el ejercicio 1.2.2. de _B.P. Lathi. Linear Sytems and Signals_:
%%
% *1.2.2. For the signal x(t) illustrated*
clear all
close all
t=-10:0.01:10;
x=@(t) abs(t).*(t>=-4&t<=2);
Figura(t,x(t),'x(t)')
axis([-5 3 -0.1 4.5])
%%
% *a. $x(t-4)$*
Figura(t,x(t-4),'x(t-4)')
axis([-1 7 -0.1 4.5])
%%
% *b. $x(\frac{t}{1.5})$*
Figura(t,x(t./1.5),'x(t/1.5)')
axis([-7 4 -0.1 4.5])
%%
% *c. $x(-t)$*
Figura(t,x(-t),'x(-t)')
axis([-3 5 -0.1 4.5])
%%
% *d. $x(2t-4)$*
Figura(t,x(2.*t-4),'x(2t-4)')
axis([-1 4 -0.1 4.5])
%%
% *e. $x(2-t)$*
Figura(t,x(2-t),'x(2-t)')
axis([-1 7 -0.1 4.5])
%% 6
%
% Para este punto se ha elaborado la funci�n _energia_, la cual recibe una
% se�al y calcula su energ�a.
%
% A manera de demostraci�n se resolver� el problema 1.1.3. de _B.P. Lathi. Linear Sytems and Signals_:
%
% *Find the energies of the pair of signals x(t) and y(t) depicted. Sketch and find
% the energies of signals x(t) + y(t) and x(t) - y(t).* 
%
% *a*
%
% $$x(t)=u(t)-u(t-2)$$
% 
% $$y(t)=u(t)-2u(t-1)+u(t-2)$$
clear all
t=-1:0.01:5;
x=@(t) 1*(heaviside(t)-heaviside(t-2));
y=@(t) (heaviside(t)-2.*heaviside(t-1)+heaviside(t-2));
close all
Figura3(t,x(t),y(t))
%%
% para $h(t)=x(t)+y(t)$:
h=@(t) (x(t)+y(t));
Figura(t,h(t),'x(t)+y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% para $h(t)=x(t)-y(t)$:
h=@(t) (x(t)-y(t));
Figura(t,h(t),'x(t)-y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% *b*
%
% $$x(t)=sen(t)(u(t)-u(t-2\pi))$$
% 
% $$y(t)=u(t)-u(t-2\pi)$$
clear all
t=-1:0.01:2.5*pi;
x=@(t) sin(t).*(heaviside(t)-heaviside(t-2*pi));
y=@(t) (heaviside(t)-heaviside(t-2*pi));
close all
Figura3(t,x(t),y(t))
%%
% para $h(t)=x(t)+y(t)$:
h=@(t) (x(t)+y(t));
Figura(t,h(t),'x(t)+y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% para $h(t)=x(t)-y(t)$:
h=@(t) (x(t)-y(t));
Figura(t,h(t),'x(t)-y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% *c*
%
% $$x(t)=sen(t)(u(t)-u(t-\pi))$$
% 
% $$y(t)=u(t)-2u(t-\pi)$$
clear all
t=-1:0.01:2*pi;
x=@(t) sin(t).*(heaviside(t)-heaviside(t-pi));
y=@(t) (heaviside(t)-heaviside(t-pi));
close all
Figura3(t,x(t),y(t))
%%
% para $h(t)=x(t)+y(t)$:
h=@(t) (x(t)+y(t));
Figura(t,h(t),'x(t)+y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% para $h(t)=x(t)-y(t)$:
h=@(t) (x(t)-y(t));
Figura(t,h(t),'x(t)-y(t)')
h=@(t) h(t).^2;
E=energia(h)
%%
% El c�digo para la funci�n _Figura3_ y para la funci�n _energia_ se
% encuentran en los ap�ndices 4 y 5 respectivamente
%% 7
%
% Para este punto se ha elaborado la funci�n _potencia_, la cual recibe una
% se�al y calcula su potencia.
%
% A manera de demostraci�n se resolver� el problema 1.1.4. de _B.P. Lathi. Linear Sytems and Signals_:
%
% *Find the power of the periodic signal $x(t)=t^3$ Find also the powers and the rms values of:.* 
clear all
t=-10:0.01:10;
T=4;
x=@(t) (t.^3).*(heaviside(t+2)-heaviside(t-2));
close all
Figura(t,x(t),'x(t)')
%%
% Aunque la funci�n es peri�dica, por la complejidad de realizar la g�fica
% en MATLAB, solo graficaremos un periodo
x=@(t) x(t).^2;
potencia(x,T)
%%
% *a* 
%
% $$x(-t)$$
clear all
t=-10:0.01:10;
T=4;
x=@(t) (t.^3).*(heaviside(t+2)-heaviside(t-2));
close all
Figura(t,x(-t),'x(-t)')
x=@(t) x(-t).^2;
potencia(x,T)
%%
% *b* 
%
% $$2x(t)$$
clear all
t=-10:0.01:10;
T=4;
x=@(t) (t.^3).*(heaviside(t+2)-heaviside(t-2));
close all
Figura(t,2*x(t),'2x(t)')
x=@(t) (2*x(t)).^2;
potencia(x,T)
%%
% El c�digo para la funci�n _potencia_  se
% encuentra en el ap�ndice 6.
%% Ap�ndice 
%
% *Ap�ndice 1* 
%
%   function [ F ] = fun1( a,w )
%   F=a/(a^2+w^2);
%   end
%
%%
% *Ap�ndice 2*
%
%   function Figura(X1, Y1, titulo)
%   %CREATEFIGURE(X1, Y1)
%   %  X1:  vector of x data
%   %  Y1:  vector of y data
% 
%   %  Auto-generated by MATLAB on 31-Mar-2019 17:32:41
% 
%   % Create figure
%   figure1 = figure;
% 
%   % Create axes
%   axes1 = axes('Parent',figure1,...
%       'Position',[0.0453879941434846 0.11 0.920937042459737 0.790702356549946]);
%   hold(axes1,'on');
% 
%   % Create plot
%   plot(X1,Y1,'ZDataSource','','DisplayName','f(x)','LineWidth',2,...
%       'Color',[1 0 0]);
% 
%   % Create title
%   title({titulo,''});
% 
%   % Set the remaining axes properties
%   set(axes1,'Color',[0.960784316062927 0.921568632125854 0.921568632125854],...
%       'FontName','Century Gothic','FontWeight','bold','XAxisLocation','origin',...
%       'XGrid','on','YAxisLocation','origin','YGrid','on');
%
%%
% *Ap�ndice 3*
%
%   function Figura2(P,X1, Y1)
%   %CREATEFIGURE(X1, Y1)
%   %  X1:  vector of x data
%   %  Y1:  vector of y data
% 
%   %  Auto-generated by MATLAB on 31-Mar-2019 19:34:23
% 
%   % Create figure
%   figure1 = figure;
% 
%   % Create subplot
%   subplot1 = subplot(2,1,1,'Parent',figure1);
%   hold(subplot1,'on');
% 
%   % Create plot
%   plot(P,X1,'Parent',subplot1,'LineWidth',2,'Color',[1 0 0]);
% 
%   % Create title
%   title('espectro de magnitud');
% 
%   % Set the remaining axes properties
%   set(subplot1,'Color',[1 0.968627452850342 0.921568632125854],'FontName',...
%       'Century Gothic','FontWeight','bold','XAxisLocation','origin','XGrid','on',...
%       'YAxisLocation','origin','YGrid','on');
%   % Create subplot
%   subplot2 = subplot(2,1,2,'Parent',figure1);
%   hold(subplot2,'on');
% 
%   % Create plot
%   plot(P,Y1,'Parent',subplot2,'LineWidth',2,'Color',[1 0 0]);
% 
%   % Create title
%   title('espectro de fase');
% 
%   % Set the remaining axes properties
%   set(subplot2,'Color',[1 0.968627452850342 0.921568632125854],'FontName',...
%       'Century Gothic','FontWeight','bold','XAxisLocation','origin','XGrid','on',...
%       'YAxisLocation','origin','YGrid','on');
%
%%
% *Ap�ndice 4*
%
%   function Figura3(P,X1, Y1)
%   %CREATEFIGURE(X1, Y1)
%   %  X1:  vector of x data
%   %  Y1:  vector of y data
% 
%   %  Auto-generated by MATLAB on 31-Mar-2019 19:34:23
% 
%   % Create figure
%   figure1 = figure;
% 
%   % Create subplot
%   subplot1 = subplot(1,2,1,'Parent',figure1);
%   hold(subplot1,'on');
% 
%   % Create plot
%   plot(P,X1,'Parent',subplot1,'LineWidth',2,'Color',[1 0 0]);
% 
%   % Create title
%   title('x(t)');
% 
%   % Set the remaining axes properties
%   set(subplot1,'Color',[1 0.968627452850342 0.921568632125854],'FontName',...
%       'Century Gothic','FontWeight','bold','XAxisLocation','origin','XGrid','on',...
%       'YAxisLocation','origin','YGrid','on');
%   axis([-1 2.5*pi -2 2])
%   % Create subplot
%   subplot2 = subplot(1,2,2,'Parent',figure1);
%   hold(subplot2,'on');
% 
%   % Create plot
%   plot(P,Y1,'Parent',subplot2,'LineWidth',2,'Color',[1 0 0]);
% 
%   % Create title
%   title('y(t)');
% 
%   % Set the remaining axes properties
%   set(subplot2,'Color',[1 0.968627452850342 0.921568632125854],'FontName',...
%       'Century Gothic','FontWeight','bold','XAxisLocation','origin','XGrid','on',...
%       'YAxisLocation','origin','YGrid','on');
%   axis([-1 2.5*pi -2 2])
%
%%
% *Ap�ndice 5*
%
%   function [ E ] = energia( X )
%   E=integral(X,-Inf,Inf);
%   end
%
%%
% *Ap�ndice 6*
%
%   function [ ] = potencia( X,T )
%   P=(integral(X,-T/2,T/2))./T
%   rms=sqrt(P)
%   end
%
%% Referencias
% 
% # B.P. Lathi. (2005). Linear Systems and Signals. New York: Oxford University Press, Inc.
% # Integral. 29 marzo 2019, de MathWorks Sitio web:
% https://es.mathworks.com/discovery/integral.html
% # Limit. 29 marzo 2019, de MathWorks Sitio web: https://www.mathworks.com/help/symbolic/limit.html
% # (2012). Juntas, pero no revueltas: sub-gr�ficas en una misma figura de Matlab . 29 marzo 2019, de 5 minutos de Matlab Sitio web: http://5minutosdematlab.blogspot.com/2012/09/juntas-pero-no-revueltas-sub-graficas.html
%


