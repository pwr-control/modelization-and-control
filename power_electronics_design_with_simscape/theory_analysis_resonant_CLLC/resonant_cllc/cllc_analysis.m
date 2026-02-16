
Rload = 10;


ZLs1 = s*hwdata.cllc.Ls1;
ZCs1 = 1/s/hwdata.cllc.Cs1;

ZLm = s*hwdata.cllc.Lm;

ZLs2 = s*hwdata.cllc.Ls2;
ZCs2 = 1/s/hwdata.cllc.Cs2;

ZLout = ZLs2 + ZCs2 + Rload;

Zpout = minreal((ZLout + ZLm)/(ZLout * ZLm));
Ht = minreal(Zpout / (ZCs1 + ZLs1 + Zpout) / ZLout * Rload);
figure; bode(Ht,options); grid on
set(gca,'xlim', [fPWM_CLLC/4 fPWM_CLLC*4])
complex_response = evalfr(Ht, 1i*2*pi*fPWM_CLLC);

% 4. Display the result
disp('Complex Response H(jw):');
disp(complex_response);

% 5. Extract Magnitude and Phase
magnitude = abs(complex_response);
phase_deg = rad2deg(angle(complex_response));

fprintf('Magnitude: %.4f\n', magnitude);
fprintf('Phase: %.2f degrees\n', phase_deg);