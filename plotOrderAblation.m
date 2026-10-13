function plotOrderAblation(x,Y,names,ylabelText,filename)
figure('Color','w','Position',[100 100 620 470]);
styles={'s-','o--','^-','d-.'};
for i=1:size(Y,2)
    plot(x,Y(:,i),styles{i},'LineWidth',2,'MarkerSize',7);hold on;
end
ax=gca;set(ax,'FontName','Times New Roman','FontSize',16, ...
    'FontWeight','normal','LineWidth',1.0,'Position',[0.13 0.15 0.84 0.66]);
grid on;ax.GridLineStyle=':';ax.GridAlpha=0.25;
xlabel('Paired channel realization');ylabel(ylabelText);
legend(names,'FontName','Times New Roman','FontSize',11, ...
    'NumColumns',2,'Location','northoutside');
saveas(gcf,filename);
end

