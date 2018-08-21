
var nav2 = {
    "pos" : [ 0.0, 0.0 ],       // meters
    "orientation" : 0.0,        // radians
    "targetOrientation" : 0.0,  // radians
    "tilt" : 0.0,               // radians
    "plan" : [],
    "gotPos" : false,
};

nav2.uniq = new Date().getTime();

nav2.runQuery = (function(cmd,callback,doneCallback) {
    var xmlHttp = new XMLHttpRequest();
    if(callback) xmlHttp.onload = callback;
    if(doneCallback) xmlHttp.onloadend = doneCallback;
    xmlHttp.open("get", "./?tt="+
        cmd+"&uniq="+nav2.uniq, true);
    nav2.uniq++;
    try {
        xmlHttp.send(null);
    } catch(z) {}
});

nav2.activeQueryCount = 0;

nav2.update=(function() {
    if(nav2.activeQueryCount==5) return;
    nav2.activeQueryCount+=3;
    nav2.runQuery('q',(function(){
        var line = this.responseText;
        var pos = new Array();
        var j=0;
        for( var i=0; i < line.length; ++i) {
            var c = line.substr(i,1);
            if( (c < '0' || c > '9') &&
                c != '.' && c != '-') {
                pos.push(parseFloat(line.substr(j,i-j)));
                if( pos.length == 3) break;
                j = i+1;
            }
        }
        if( pos.length == 3) {
            nav2.pos[0] = pos[0];
            nav2.pos[1] = pos[1];
            nav2.orientation = pos[2] * Math.PI / 180;
            if(!nav2.gotPos) {
                nav2.targetOrientation = nav2.orientation;
                nav2.gotPos = true;
            }
        }
    }),(function(){nav2.activeQueryCount--}));
    nav2.runQuery('qtilt',(function(){
        nav2.tilt = parseFloat(this.responseText) * Math.PI / 180;
    }),(function(){nav2.activeQueryCount--}));
    nav2.runQuery('plan',(function(){
        var re = /\[([-+e.0-9]+),([-+e.0-9]+)\]:r=([-+e.0-9]+);o=([-+e.0-9]+)/g;
        var match;
        nav2.plan = [];
        while(match = re.exec(this.responseText)) {
            nav2.plan.push( {x:Number(match[1]),y:Number(match[2]),
                r:Number(match[3]),o:Number(match[4])});
        }
    }),(function(){nav2.activeQueryCount--}));
});

nav2.stop=(function() {
    nav2.runQuery("s");
});

nav2.setTargetOrientation=(function(orientation)
{
    nav2.runQuery("o%20"+(orientation*180/Math.PI));
    nav2.targetOrientation = orientation;
});

nav2.setTargetVelocity=(function(velocity)
{
    nav2.runQuery("av%20"+velocity[0]+"%20"+velocity[1]);
});

nav2.setPosition=(function(pos,orientation) {
    nav2.runQuery("p%20"+pos[0]+"%20"+pos[1]+
        "%20"+(orientation*180/Math.PI));
});

nav2.setTargetHeadTilt=(function(tilt) {
    nav2.runQuery("tilt%20"+(tilt*180/Math.PI));
});

nav2.setTargetPath=(function(path) {
    nav2.runQuery("s");
    for( var i=1; i < path.length; ++i) {
        var dx = path[i][0] - path[i-1][0];
        var dy = path[i][1] - path[i-1][1];
        var relAngle = Math.atan2(dy,dx) - nav2.orientation;
        var dist = Math.sqrt(dx*dx+dy*dy);
        nav2.runQuery("mv%20"+dist+"%20"+(relAngle*180/Math.PI));
    }
});
