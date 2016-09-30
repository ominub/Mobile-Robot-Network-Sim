writerObj = VideoWriter('movie.mp4','MPEG-4');
writerObj.FrameRate =6;
writerObj.Quality=100;
writeObj.CompressionRatio=40;
open(writerObj);
ss=3;
for tt=1:100
        handles=waitbar(tt/(361));
        fname=sprintf('%d.jpg',tt);
        filename=['D:\Coverage\MRN\',fname];
        im=imread(filename);
        writeVideo(writerObj,im);
end
close(writerObj);
delete(handles)