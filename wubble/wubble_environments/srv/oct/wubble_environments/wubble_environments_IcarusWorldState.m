% Auto-generated.  Do not edit!

% [reqmsg,resmsg] = core_testsrv1()
%
function [reqmsg,resmsg] = wubble_environments_IcarusWorldState()
if( nargout > 0 )
    reqmsg = wubble_environments_Request();
end
if( nargout > 0 )
    resmsg = wubble_environments_Response();
end

% Auto-generated.  Do not edit!

% msg = wubble_environments_Request()
%
% Request message type, fields include:

% //! \htmlinclude Request.msg.html
function msg = wubble_environments_Request()

msg = [];
msg.create_response_ = @wubble_environments_Response;
msg.md5sum_ = @wubble_environments_Request___md5sum;
msg.server_md5sum_ = @wubble_environments_Request___server_md5sum;
msg.server_type_ = @wubble_environments_Request___server_type;
msg.type_ = @wubble_environments_Request___type;
msg.serializationLength_ = @wubble_environments_Request___serializationLength;
msg.serialize_ = @wubble_environments_Request___serialize;
msg.deserialize_ = @wubble_environments_Request___deserialize;
msg.message_definition_ = @wubble_environments_Request___message_definition;

function x = wubble_environments_Request___md5sum()
x = '';

function x = wubble_environments_Request___server_md5sum()
x = 'af6d3a99f0fbeb66d3248fa4b3e675fb';

function x = wubble_environments_Request___server_type()
x = '';

function x = wubble_environments_Request___message_definition()
x = [    '\n' ...
];

function x = wubble_environments_Request___type()
x = 'wubble_environments/IcarusWorldStateRequest';

function l__ = wubble_environments_Request___serializationLength(msg)
l__ = 0;

function dat__ = wubble_environments_Request___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wubble_environments_Request___deserialize(dat__, fid__)
msg__ = wubble_environments_Request();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
if( file_created__ )
    fclose(fid__);
end
function l__ = wubble_environments_Request___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

% msg = wubble_environments_Response()
%
% Response message type, fields include:
% string state

% //! \htmlinclude Response.msg.html
function msg = wubble_environments_Response()

msg = [];
msg.state = '';
msg.md5sum_ = @wubble_environments_Response___md5sum;
msg.server_md5sum_ = @wubble_environments_Response___server_md5sum;
msg.server_type_ = @wubble_environments_Response___server_type;
msg.type_ = @wubble_environments_Response___type;
msg.serializationLength_ = @wubble_environments_Response___serializationLength;
msg.serialize_ = @wubble_environments_Response___serialize;
msg.deserialize_ = @wubble_environments_Response___deserialize;
msg.message_definition_ = @wubble_environments_Response___message_definition;

function x = wubble_environments_Response___md5sum()
x = '';

function x = wubble_environments_Response___server_md5sum()
x = 'af6d3a99f0fbeb66d3248fa4b3e675fb';

function x = wubble_environments_Response___server_type()
x = '';

function x = wubble_environments_Response___message_definition()
x = [    '\n' ...
];

function x = wubble_environments_Response___type()
x = 'wubble_environments/IcarusWorldStateResponse';

function l__ = wubble_environments_Response___serializationLength(msg)
l__ =  ...
    + 4 + numel(msg.state);

function dat__ = wubble_environments_Response___serialize(msg__, seq__, fid__)
global rosoct

c__ = 0;
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
end
fwrite(fid__, numel(msg__.state), 'uint32');
fwrite(fid__, msg__.state, 'uint8');
if( file_created__ )
    fseek(fid__,0,SEEK_SET);
    dat__ = fread(fid__,Inf,'uint8=>uint8');
    fclose(fid__);
end

function msg__ = wubble_environments_Response___deserialize(dat__, fid__)
msg__ = wubble_environments_Response();
file_created__ = 0;
if( ~exist('fid__','var') )
    fid__ = tmpfile();
    file_created__ = 1;
    fwrite(fid__,dat__,'uint8');
    fseek(fid__,0,SEEK_SET);
end
size__ = double(fread(fid__, 1,'uint32=>uint32'));
msg__.state = fread(fid__, size__, '*char')';
if( file_created__ )
    fclose(fid__);
end
function l__ = wubble_environments_Response___sum_array_length__(x)
if( ~exist('x','var') || isempty(x) )
    l__ = 0;
else
    l__ = sum(x(:));
end

