package lib;

import ilog.concert.IloException;
import problem.InstanceIO;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;

public class Readers {

    public static List<Path> getDirectories(String directoryPath) {
        List<Path> directories = new ArrayList<>();
        Path directory = Paths.get(directoryPath);

        try {
            // Iterate over the directory's contents
            Files.list(directory)
                    .filter(Files::isDirectory) // Filter only directories
                    .forEach(directories::add);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return directories;
    }

    public static List<Path> getFiles(String directoryPath) {
        List<Path> files = new ArrayList<>();

        Path directory = Paths.get(directoryPath);

        System.out.println("Reading Files from Input Directory " + directory.toString());

        try {
            // Iterate over the directory's contents
            Files.list(directory)
                    .filter(Files::isRegularFile) // Filter only files
                    .forEach(files::add);
        } catch (IOException e) {
            e.printStackTrace();
        }
        return files;
    }

    public static void readInstance(Path file, int towerNumber, int fleetSize) throws IloException, IOException {
        String instanceName = file.getFileName().toString();
        DateTimeFormatter formatter = DateTimeFormatter.ofPattern("yyMMdd.HHmmss");
        String formattedDateTime = LocalDateTime.now().format(formatter);
        System.out.println("Current Time:"+formattedDateTime);
        System.out.println("Reading instance: " + instanceName);
        // Check if the directory listing is not null
        InstanceIO io = new InstanceIO();
        io.readInstance(file.toAbsolutePath().toString(),towerNumber,fleetSize);
    }
}
