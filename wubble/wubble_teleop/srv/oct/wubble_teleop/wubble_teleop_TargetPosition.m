% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = wubble_teleop_TargetPosition()
if( nargout > 0 )
    reqmsg = wubble_teleop_Request();
end
if( nargout > 0 )
    resmsg = wubble_teleop_Response();
end

% Auto-generated.  Do not edit!

% msg = wubble_teleop_Request()
%
% Request message type, fields include:
% double x
% double y
% double z

% //! \htmlinclude Request.msg.html
function msg = wubble_teleop_Request()

msg = [];
msg.create_response_ = @wubble_teleop_Response;
msg.x = double(0);
msg.y = double(0);
msg.z = double(0);
msg.md5sum_ = @wubble_teleop_Request___md5sum;
msg.server_md5sum_ = @wubble_teleop_Request___server_md5sum;
msg.server_type_ = @wubble_teleop_Request___server_type;
msg.type_ = @wubble_teleop_Request___type;
msg.serializationLength_ = @wubble_teleop_Request___serializationLength;
msg.serialize_ = @wubble_teleop_Request___serialize;
msg.deserialize_ = @wubble_teleop_Request___deserialize;
msg.message_definition_ = @wubble_teleop_Request___message_definition;

function x = wubble_teleop_Request___md5sum()
x = '';

function x = wubble_teleop_Request___server_md5sum()
x = '14fb54e9e518f55d418823395ca25d0b';

function x = wubble_teleop_Request___server_type()
x = '';

function x = wubble_teleop_Request___message_definition()
x = [    '\n' ...
];

function x = wubble_teleop_Request___type()
x = 'wubble_teleop/TargetPositionRequest';

function l__ = wubble_teleop_Request___serializationLength(msg)
l__ =  ...
    + 8 ...
    + 8 ...
    + 8;

function dat__ = wubble_teleop_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.x, 'double');
c__ = c__ + fwrite(fid__, msg__.y, 'double');
c__ = c__ + fwrite(fid__, msg__.z, 'double');
if( c__ ~= 3 )
    error('some members of msg wubble_teleop:Request are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wubble_teleop_Request___deserialize(dat__, fid__)
msg__ = wubble_teleop_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.x = fread(fid__,1,'double=>double');
msg__.y = fread(fid__,1,'double=>double');
msg__.z = fread(fid__,1,'double=>double');
if( file_created__ )
    fclose(fid__);
end
function l__ = wubble_teleop_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = wubble_teleop_Response()
%
% Response message type, fields include:
% uint8 success

% //! \htmlinclude Response.msg.html
function msg = wubble_teleop_Response()

msg = [];
msg.success = uint8(0);
msg.md5sum_ = @wubble_teleop_Response___md5sum;
msg.server_md5sum_ = @wubble_teleop_Response___server_md5sum;
msg.server_type_ = @wubble_teleop_Response___server_type;
msg.type_ = @wubble_teleop_Response___type;
msg.serializationLength_ = @wubble_teleop_Response___serializationLength;
msg.serialize_ = @wubble_teleop_Response___serialize;
msg.deserialize_ = @wubble_teleop_Response___deserialize;
msg.message_definition_ = @wubble_teleop_Response___message_definition;

function x = wubble_teleop_Response___md5sum()
x = '';

function x = wubble_teleop_Response___server_md5sum()
x = '14fb54e9e518f55d418823395ca25d0b';

function x = wubble_teleop_Response___server_type()
x = '';

function x = wubble_teleop_Response___message_definition()
x = [    '\n' ...
];

function x = wubble_teleop_Response___type()
x = 'wubble_teleop/TargetPositionResponse';

function l__ = wubble_teleop_Response___serializationLength(msg)
l__ =  ...
    + 1;

function dat__ = wubble_teleop_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
c__ = c__ + fwrite(fid__, msg__.success, 'uint8');
if( c__ ~= 1 )
    error('some members of msg wubble_teleop:Response are initialized incorrectly!');
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wubble_teleop_Response___deserialize(dat__, fid__)
msg__ = wubble_teleop_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
msg__.success = fread(fid__,1,'uint8=>uint8');
if( file_created__ )
    fclose(fid__);
end
function l__ = wubble_teleop_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

