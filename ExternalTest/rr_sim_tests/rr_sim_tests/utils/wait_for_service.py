#! /usr/bin/env python3
# Copyright 2020-2021 Rapyuta Robotics Co., Ltd.

def canonical_name(name, sep='/'):
    if not name or name == sep: 
        return name 
    elif name[0] == sep: 
        return '/' + '/'.join([x for x in name.split(sep) if x]) 
    else: 
        return '/'.join([x for x in name.split(sep) if x])    

def wait_for_service(in_node, in_service, in_service_name, in_timeout=5.0):
    service_name = canonical_name(in_service_name)
    print(f'Waiting for service [{service_name}]...')
    cli = in_node.create_client(in_service, service_name)
    if cli.wait_for_service(timeout_sec=in_timeout):
        print(f'Service [{service_name}]: ready')
    else:
        in_node.get_logger().error(f'{in_service}[{service_name}] service not available')
    return cli
