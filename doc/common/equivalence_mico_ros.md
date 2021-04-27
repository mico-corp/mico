Equivalence MICO-ROS {#equivalence_mico_ros}
=====================

As middlewares and robotics frameworks, MICO and ROS share some apparent similarities in the concepts (focusing mainly in the communication side of the frameworks). In ROS the streams of data are typically shared using *Publishers* and *Subscribers*. In MICO, there exists *Pipes* and *Policies*. Here we can see some pieces of code side by side for each of the libraries.

-------------------------------------
<table border="1">
 <tr>
    <td><b style="font-size:30px">MICO</b></td>
    <td><b style="font-size:30px">ROS</b></td>
 </tr>
 <tr valign="top">
    <td>
        <code>
            ThreadPool::init(4);<br><br><br><br><br>
            auto pol1 = new Policy( { makeInput<int>("pol1") } );<br>
            pol1->registerCallback({"pol1"},  [&](DataFlow _data){      ...   });<br><br>
            auto pipe1 = new Outpipe("pol1", typeid(int).name());<br>
            pipe1->registerPolicy(pol1, "pol1");<br><br>
            for(int i = 0 ; i < nIters ; i++) pipe1->flush(i); <br>
        </code>
    </td>
    <td>
        <code>
            ros::init( _argc, _argv, "benchmark_ros");<br>
            ros::AsyncSpinner spin(4);<br>
            spin.start();<br>
            ros::NodeHandle nh;<br><br>
            auto sub1  =  nh.subscribe<std_msgs::Int32>("topic", 0, [&](const std_msgs::Int32 &_data){ ... };);<br><br><br>
            auto pub1 = nh.advertise<std_msgs::Int32>("topic", 0);<br><br><br>
            for(int i = 0 ; i < nIters ; i++) pub1.publish(std_msgs::Int32(i));
        </code>
    </td>
 </tr>
</table>

-------------------------------------

The syntax differs but the concept of broadcasters of data and listeners of data is similar in both cases. Nevertheless, the underneath implementation is completely different in both libraries. ROS uses a low-level layer of sockets that communicates with a "master" process independent of the process running the *publishers* and *subscribers*. This complicated but robust and widely tested architecture provides for an easy way of creating different application that communicates between each other. Contrary, MICO has been designed to be everything inside of the same process. The selection of this architecture has some advantages, but also disadvantages. 

The main advantage of this architecture concerns the speed and data copying. Because everything happens within the same processes, the memory is shared across the whole system. Thus, there is no need to use any kind of data parsing algorithm, protobuffers or streaming bytes arrays. Instead, the same objects that are created in one part of the application is shared "as it is", saving time and reducing copying of data.

However, the main drawback is that when the system runs, it does in a monolithic manner. It means, that everything happens within the same process and unhandled exceptions and crashes affect to the whole system.

In terms of coding and modularity, MICO offers a simple interface helps users to create new modular and reusable code. In ROS, applications are usually called nodes, each node is designed to perform an specific task. In MICO, there are blocks. Blocks are independent modules that are designed to perform an specific task. Blocks are mainly composed by N input Policies and M output tags. Then the block reacts differently according to the inputs. 

To learn how to create a new block go to the tutorial "Create a new MICO block".