function resp = fcn_service(~, req, resp)
disp('Service called!');
resp.Sum = req.A + req.B;
end

