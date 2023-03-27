classdef LockClamp < handle
    properties
        series
        type
        lengthType
        pointCloud
        xmin
        xmax
        ymin
        ymax
        zmin
        zmax
        surfaceX
        surfaceY
        surfaceZ
        panel
        paintingBase
        area
    end
    methods
        function obj = LockClamp(series,pointCloud)
            obj.series = series;
            obj.pointCloud = pcdenoise(pointCloud);
            obj.xmin = min(obj.pointCloud.Location(:,1));
            obj.xmax = max(obj.pointCloud.Location(:,1));
            obj.ymin = min(obj.pointCloud.Location(:,2));
            obj.ymax = max(obj.pointCloud.Location(:,2));
            [value,idx] = max(obj.pointCloud.Location(:,1));
            obj.zmin = min(obj.pointCloud.Location(:,3));
            obj.zmax = max(obj.pointCloud.Location(:,3));
            range = 100;
            [row,~] = find(obj.pointCloud.Location(:,1)>obj.xmax - range & obj.pointCloud.Location(:,1)<obj.xmax + range);
            xSelectedPoints = obj.pointCloud.Location(row,:);
            [row,~] = find(obj.pointCloud.Location(:,2)>obj.ymax - range & obj.pointCloud.Location(:,2)<obj.ymax + range);
            ySelectedPoints = obj.pointCloud.Location(row,:);
            [c, ~, ~] = intersect(xSelectedPoints,ySelectedPoints,'rows');
            zmaxList = sortrows(c,3,'descend');


            hight = obj.zmax - obj.zmin;
            if(hight> 500)
                obj.type = "下半件";
            else
                obj.type = "上半件";
            end


            length = obj.ymax - obj.ymin;
            if(length> 1200)
                obj.lengthType = "long";
            else
                obj.lengthType = "short";
            end


            if size(zmaxList,1)>10
            obj.panel = zmaxList(10,3);
            else
            obj.panel = zmaxList(1,3);
            end



            outline = [obj.xmin,obj.ymin,obj.panel;obj.xmin,obj.ymax,obj.panel;obj.xmax,obj.ymax,obj.panel;obj.xmax,obj.ymin,obj.panel];
            A=outline(1,:)';
            B=outline(2,:)';
            C=outline(3,:)';
            D=outline(4,:)';
            P = [B,A;C,D];
            obj.surfaceX = P([1,4],:);
            obj.surfaceY = P([2,5],:);
            obj.surfaceZ = P([3,6],:);
            if obj.ymax-obj.ymin<=1200
                if obj.type =="上半件"
                    %左侧立面
                    obj.paintingBase = [obj.xmin,(obj.ymax+obj.ymin)/2];
                    %上端面
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 400,obj.ymin-400];
                    %右侧立面
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 600, (obj.ymax+obj.ymin)/2];
                    %上表面1
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 200, obj.ymin - 400];
                    %端面下
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 300, obj.ymax];
                    %
                    %obj.paintingBase = [obj.paintingBase;obj.xmax + 200, obj.ymax+400];
                else 
                   %上表面1
                    obj.paintingBase = [obj.xmin - 500, obj.ymin - 300];
                    %左侧立面
                     obj.paintingBase = [obj.paintingBase;obj.xmin,  (obj.ymax+obj.ymin)/2];
                     %下端面
                     obj.paintingBase = [obj.paintingBase;obj.xmax + 300,  obj.ymax];
                     %右侧立面
                     obj.paintingBase = [obj.paintingBase;obj.xmax + 400,  (obj.ymax+obj.ymin)/2];
                     %上表面2
                     obj.paintingBase = [obj.paintingBase;obj.xmax + 1000,  obj.ymin - 400];
                     %上端面
                     obj.paintingBase = [obj.paintingBase;obj.xmax + 400,  obj.ymin - 400];

                end
            else %>1200
                if obj.type =="上半件"
                    %左侧立面1
                    obj.paintingBase = [obj.xmin, (obj.ymax+obj.ymin*3)/4];
                    %上端面
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 400,obj.ymin - 400];
                    %右侧1
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 400,(obj.ymax + obj.ymin*3)/4];
                    %上表面1
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 200, obj.ymin-400];
                    %上表面2
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 200, (obj.ymax+obj.ymin)/2 - 100];
                    %下端面
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 300, obj.ymax];
                    %左侧立面2
                    obj.paintingBase = [obj.paintingBase;obj.xmin, (obj.ymin + obj.ymax*3)/4];
                    %右侧立面2
                    obj.paintingBase = [obj.paintingBase;obj.xmax + 400, (obj.ymin + obj.ymin*3)/4];

                else %》1500的下半件
                    %上表面1
                    obj.paintingBase = [obj.xmin-500,obj.ymin-300];
                    %左侧1
                    obj.paintingBase = [obj.paintingBase; obj.xmin,  (obj.ymin + obj.ymax*3)/4];
                    %上表面2
                    obj.paintingBase = [obj.paintingBase; obj.xmin-500, (obj.ymax+obj.ymin)/2 - 450];
                    %下端面
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 300, obj.ymax];
                    %上表面3
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 1000, obj.ymin - 500];
                    %右侧1
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 400, (obj.ymin + obj.ymax*3)/4];
                    %上表面4
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 1000, (obj.ymax+obj.ymin)/2 - 300];
                    %上端面
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 400,obj.ymin - 400];
                    %左侧2
                    obj.paintingBase = [obj.paintingBase; obj.xmin,(obj.ymin + obj.ymax*3)/4];
                    %右侧2
                    obj.paintingBase = [obj.paintingBase; obj.xmax + 400,(obj.ymin + obj.ymax*3)/4];

                end
            end
        end
    end
end
