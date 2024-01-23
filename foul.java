import java.io.File;

public class foul {

    public static void main(String[] args) {
        // Provide the path to the directory you want to process
        String directoryPath = "path/to/your/directory";

        // Call the method to unpack and delete empty folders
        unpackAndDeleteEmptyFolders(new File(directoryPath));

        System.out.println("Unpacking and deleting empty folders completed.");
    }

    public static void unpackAndDeleteEmptyFolders(File directory) {
        // Check if the directory is valid
        if (directory.exists() && directory.isDirectory()) {
            // Get list of files and subdirectories
            File[] files = directory.listFiles();

            // Process each file or subdirectory
            if (files != null) {
                for (File file : files) {
                    if (file.isDirectory()) {
                        // Recursively process subdirectories
                        unpackAndDeleteEmptyFolders(file);

                        // After processing the subdirectory, check if it's empty and delete it
                        if (file.list().length == 0) {
                            System.out.println("Deleting empty directory: " + file.getAbsolutePath());
                            file.delete();
                        }
                    }
                }
            }
        }
    }
}
