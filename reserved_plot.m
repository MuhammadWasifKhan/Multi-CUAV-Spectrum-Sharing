function reserved_plot(x,Y,names,ylab,filename)
figure('Position',[100 100 620 470]);
styles={'o-','s--','d-.','^-','v:'};hold on;
for ii=1:size(Y,2)
    plot(x,Y(:,ii),styles{ii},'LineWidth',2,'MarkerSize',7);
end
ax=gca;ax.FontName='Times New Roman';ax.FontSize=16;
ax.FontWeight='normal';ax.LineWidth=1.2;
ax.Position=[0.13 0.13 0.84 0.68];grid on;
ax.GridLineStyle=':';ax.GridAlpha=0.30;
xlabel('Paired channel realization');ylabel(ylab);
legend(names,'FontName','Times New Roman','FontSize',12, ...
    'NumColumns',2,'Location','northoutside');
saveas(gcf,filename);
end


%% ===== FINAL RAW-INTERFERENCE RESERVED-SUBCHANNEL CONTROLLER =====
