Data=xlsread('Boiling_Data.xlsx','Test 26032025','D2:F39');

save_folder = 'Figures';
if ~exist(save_folder, 'dir'), mkdir(save_folder); end
Npts=size(Data,1);

Dw=100.0e-6;    delDw=1.0e-6;
Lw=60.0e-3;     delLw=0.5e-3;  
delTwater=1.0;
rho_0=9.847e-8;
alpha=3.744e-10;

Results=zeros(Npts,15);
for k1=1:1:Npts
    V=Data(k1,1);       delV=0.01+0.01*V;
    I=Data(k1,2);       delI=0.01+0.01*I;
    
    rho=0.25*pi*Dw^2*V/(Lw*I);
    delrho=rho*sqrt(4*(delDw/Dw)^2+(delLw/Lw)^2+(delV/V)^2+(delI/I)^2);
    
    Twire=(rho-rho_0)/alpha;
    delTwire=delrho/alpha;
    
    W=V*I;
    delW=W*sqrt((delV/V)^2+(delI/I)^2);
    
    Flux=W/(pi*Dw*Lw);
    delFlux=Flux*sqrt((delW/W)^2+(delDw/Dw)^2+(delLw/Lw)^2);
    
    Twater=Data(k1,3);
    HTC=Flux/(Twire-Twater);
    delHTC=HTC*sqrt((delFlux/Flux)^2+...
        (sqrt(delTwire^2+delTwater^2)/(Twire-Twater))^2);
    
    Results(k1,:)=[V,delV,I,delI,rho,delrho,Twire,delTwire,...
                   W,delW,Flux,delFlux,Twater,HTC,delHTC];
end

% Fig 1: Voltage
figure(1)
plot(Results(:,1),'o','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Data Point Number');
ylabel('Voltage (V)');
yl=ylim; ylim([0, yl(2)]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig01_Voltage'), '-dpng', '-r300');

% Fig 2: Current
figure(2)
plot(Results(:,3),'o','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Data Point Number');
ylabel('Current (A)');
yl=ylim; ylim([0, yl(2)]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig02_Current'), '-dpng', '-r300');

% Fig 3: Water Temperature
figure(3)
plot(Results(:,13),'o','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Data Point Number');
ylabel('Water Temperature (C)');
yl=ylim; ylim([20, 30]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig03_WaterTemperature'), '-dpng', '-r300');

% Fig 4: Voltage uncertainty %
figure(4)
plot(Results(:,1),100.*Results(:,2)./Results(:,1),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Voltage (V)');
ylabel('Uncertainty of Voltage (%)');
yl=ylim; ylim([0, 4]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig04_VoltageUncertainty'), '-dpng', '-r300');

% Fig 5: Current uncertainty %
figure(5)
plot(Results(:,3),100.*Results(:,4)./Results(:,3),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Current (A)');
ylabel('Uncertainty of Current (%)');
yl=ylim; ylim([0, 4]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig05_CurrentUncertainty'), '-dpng', '-r300');

% Fig 6: Resistivity uncertainty %
figure(6)
plot(Results(:,5),100.*Results(:,6)./Results(:,5),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Resistivity (\Omega m)');
ylabel('Uncertainty of Resistivity (%)');
yl=ylim; ylim([0, 4]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig06_ResistivityUncertainty'), '-dpng', '-r300');

% Fig 7: Wire Temp uncertainty %
figure(7)
plot(Results(:,7),100.*Results(:,8)./Results(:,7),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Wire Temperature (C)');
ylabel('Uncertainty of Wire Temperature (%)');
yl=ylim; ylim([0, 25]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig07_WireTempUncertaintyPct'), '-dpng', '-r300');

% Fig 8: Wire Temp uncertainty (absolute)
figure(8)
plot(Results(:,7),Results(:,8),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Wire Temperature (C)');
ylabel('Uncertainty of Wire Temperature (C)');
yl=ylim; ylim([0, 25]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig08_WireTempUncertaintyAbs'), '-dpng', '-r300');

% Fig 9: Wire vs Water Temperature
figure(9)
plot(Results(:,7),'o','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','r'); hold on;
plot(Results(:,13),'s','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','b');
grid on;
xlabel('Data Point Number');
ylabel('Temperature (C)');
legend('Wire','Water','location','northwest');
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig09_WireWaterTemperature'), '-dpng', '-r300');

% Fig 10: Power uncertainty %
figure(10)
plot(Results(:,9),100.*Results(:,10)./Results(:,9),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Power (W)');
ylabel('Uncertainty of Power (%)');
yl=ylim; ylim([0, 4]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig10_PowerUncertainty'), '-dpng', '-r300');

% Fig 11: Heat Flux uncertainty %
figure(11)
plot(1e-6.*Results(:,11),100.*Results(:,12)./Results(:,11),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('Heat Flux (MW/m^2)');
ylabel('Uncertainty of Heat Flux (%)');
yl=ylim; ylim([0, 4]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig11_HeatFluxUncertainty'), '-dpng', '-r300');

% Fig 12: HTC uncertainty %
figure(12)
plot(1e-3.*Results(:,14),100.*Results(:,15)./Results(:,14),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r');
grid on;
xlabel('HTC (kW/m^2K)');
ylabel('Uncertainty of HTC (%)');
yl=ylim; ylim([0, 25]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig12_HTCUncertainty'), '-dpng', '-r300');

% Fig 13: Boiling Curve (Power) with error bars 
figure(13)
errorbar(Results(:,7),Results(:,9),Results(:,10),Results(:,10),...
    Results(:,8),Results(:,8),'o','MarkerEdgeColor','k',...
    'MarkerFaceColor','r','CapSize',0,'Color','r');
grid on;
xlabel('Wire Temperature (C)');
ylabel('Power (W)');
yl=ylim; ylim([0, yl(2)]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig13_BoilingCurve_Power'), '-dpng', '-r300');

% Fig 14: Boiling Curve (Heat Flux) with error bars 
figure(14)
errorbar(Results(:,7),1e-6.*Results(:,11),1e-6.*Results(:,12),...
    1e-6.*Results(:,12),Results(:,8),Results(:,8),'o',...
    'MarkerEdgeColor','k','MarkerFaceColor','r','CapSize',0,'Color','r');
grid on;
xlabel('Wire Temperature (C)');
ylabel('Heat Flux (MW/m^2)');
yl=ylim; ylim([0, yl(2)]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig14_BoilingCurve_HeatFlux'), '-dpng', '-r300');

% Fig 15: HTC with error bars 
figure(15)
errorbar(Results(:,7)-Results(:,13),1e-3.*Results(:,14),...
    1e-3.*Results(:,15),1e-3.*Results(:,15),...
    sqrt(Results(:,8).^2+delTwater^2),sqrt(Results(:,8).^2+delTwater^2),...
    'o','MarkerSize',8,'MarkerEdgeColor','k','MarkerFaceColor','r','CapSize',0,'Color','r');
grid on;
xlabel('Wire-Water Temperature Difference (C)');
ylabel('HTC (kW/m^2K)');
yl=ylim; ylim([0, yl(2)]);
set(gca,'FontSize',12);
set(gcf,'Units','centimeters','Position',[0 0 12 9]);
print(gcf, fullfile(save_folder, 'Fig15_BoilingCurve_HTC'), '-dpng', '-r300');