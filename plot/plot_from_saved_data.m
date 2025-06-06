saved_data_path = "\\Ws2023\ws2025\Work2025\YosukeKOSEKI\gain60_zref0.3_Log(03-Jun-2025_13_02_08).mat"; % 描画したいデータのフルパス
Data = load(saved_data_path);

new_Data = simplifyLogger(Data.log, 1);
plot(new_Data.t, new_Data.estimator.p, 'LineWidth', 2)
hold on
plot(new_Data.t, new_Data.reference.p, '--', 'LineWidth', 2)
xlabel('t [s]')
ylabel('position [m]')
legend('xe', 'ye', 'xe', 'xr', 'yr', 'zr', 'Location', 'bestoutside')
grid on
ax = gca;
ax.FontSize = 20;

saveas(gcf, 'gain60_zref0_3')