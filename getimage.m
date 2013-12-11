function img = getimage(url, fmt_s)

fmt = imformats(fmt_s);

if isempty(fmt)
    error('invalid image format');
end

img_data = urlread2(url);

fname = ['tmp.' fmt_s];
fid = fopen(fname, 'w');
fwrite(fid, img_data);
fclose(fid);

img = fmt.read(fname);

end

